//! 守卫循环：导航调度器的输入准入、目标/路径/取消处理与状态发布。

use super::*;

#[cfg(feature = "guard-runtime")]
pub(crate) fn guarded_nav_scheduler_owner(node: &TaskChannels, body_reference_frame_id: Arc<str>) -> NavStackResult<TaskThread> {
    let node = node.clone();
    spawn_task("guarded-nav-scheduler", Box::new(move |shutdown, ready| {
        Box::pin(run_guarded_nav_scheduler(node, body_reference_frame_id, shutdown, ready))
    }))
}

#[cfg(feature = "guard-runtime")]
#[derive(Clone, Debug, Default, PartialEq)]
pub(super) struct GuardedNavTelemetry {
    pub(super) runtime_generation: u64,
    pub(super) public_session_id: String,
    pub(super) motion_generation: i64,
    pub(super) map_frame: String,
    pub(super) pose_frame: String,
    pub(super) goal: [f64; 3],
    pub(super) current: [f64; 3],
    // 旧线格式的第三项仍为yaw；三维高度在内部独立保留用于真实进度。
    pub(super) current_z: f64,
    pub(super) goal_z: f64,
    pub(super) surface_navigation: bool,
    pub(super) initial_distance: f64,
    pub(super) progress: f64,
    pub(super) path_revision: i64,
    pub(super) planner_runtime_generation: i64,
}

#[cfg(feature = "guard-runtime")]
pub(super) fn guarded_goal_execution_identity(
    goal: &schema::NavigationGoal,
) -> nuwax_nav_runtime::PathExecutionIdentity {
    nuwax_nav_runtime::PathExecutionIdentity { goal_key: goal.goal_key(), session_id: goal.session_id.clone(), planner_runtime_generation: 0, path_revision: 0 }
}

#[cfg(feature = "guard-runtime")]
pub(super) fn guarded_path_execution_identity(
    path: &schema::PathEnvelope,
) -> nuwax_nav_runtime::PathExecutionIdentity {
    nuwax_nav_runtime::PathExecutionIdentity {
        goal_key: path.goal_key(),
        session_id: path.session_id.clone(),

        planner_runtime_generation: path.planner_runtime_generation,
        path_revision: u64::try_from(path.path_revision).expect("validated positive path revision"),
    }
}

#[cfg(feature = "guard-runtime")]
pub(super) struct GuardedPoseInput {
    pub(super) pose: PoseStamped,
    pub(super) frame_id: String,
    pub(super) current: [f64; 3],
}

#[cfg(feature = "guard-runtime")]
pub(super) fn latest_pose_frame(
    first: Option<TransportResult<RawLcmFrame>>,
    poses: &mut LcmFrameStream,
    name: &str,
) -> NavStackResult<RawLcmFrame> {
    let mut latest = required_frame(first, name)?;
    while let Some(next) = poses.next().now_or_never() {
        latest = required_frame(next, name)?;
    }
    Ok(latest)
}

#[cfg(feature = "guard-runtime")]
pub(super) fn drain_guarded_pose_input(
    first: Option<TransportResult<RawLcmFrame>>,
    poses: &mut LcmFrameStream,
    diagnostics: &mut InputDiagnosticLimiter,
) -> NavStackResult<Option<GuardedPoseInput>> {
    let frame = latest_pose_frame(first, poses, "guarded_nav_scheduler.map_pose")?;
    match decode_guarded_pose_input(&frame.payload)? {
        OwnerInput::Accepted(candidate) => Ok(Some(candidate)),
        OwnerInput::Rejected(reason) => {
            diagnostics.reject("guarded_nav_scheduler.map_pose", &reason);
            Ok(None)
        }
    }
}

#[cfg(feature = "guard-runtime")]
pub(super) fn configured_pose_source_max_age() -> NavStackResult<Duration> {
    let seconds = nuwax_config::geometry()
        .map_err(NavStackError::InvalidConfig)?
        .odom_max_age_sec;
    Duration::try_from_secs_f64(seconds)
        .ok()
        .filter(|value| !value.is_zero())
        .ok_or_else(|| {
            NavStackError::InvalidConfig("NUWAX_GEOMETRY_ODOM_MAX_AGE_SEC必须为正有限秒数".into())
        })
}

#[cfg(feature="guard-runtime")]
pub(super) fn decode_guarded_pose_input(payload: &[u8]) -> OwnerInputResult<GuardedPoseInput> {
    let pose_message =
        match schema::decode_pose_stamped(&prefixed(schema::POSE_STAMPED_FINGERPRINT, payload)) {
            Ok(message) => message,
            Err(error) => {
                return Ok(OwnerInput::Rejected(format!("map pose decode: {error}")));
            }
        };
    if pose_message.header.frame_id != "map" {
        return Ok(OwnerInput::Rejected(format!(
            "scheduler map pose frame must be map, got {}",
            pose_message.header.frame_id,
        )));
    }
    if let Err(error) =
        lcm_header_timestamp_ns(&pose_message.header, "guarded_nav_scheduler.map_pose")
    {
        return Ok(OwnerInput::Rejected(error.to_string()));
    }
    if !schema_pose_is_finite_and_normalized(&pose_message.pose) {
        return Ok(OwnerInput::Rejected(
            "scheduler map pose contains non-finite values or a zero-norm quaternion".into(),
        ));
    }
    let pose = match pose_from_schema(&pose_message) {
        Ok(pose) => pose,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let current = [
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.orientation.yaw(),
    ];
    Ok(OwnerInput::Accepted(GuardedPoseInput {
        pose,
        frame_id: pose_message.header.frame_id,
        current,
    }))
}

#[cfg(feature = "guard-runtime")]
pub(super) struct GuardedCostmapInput {
    pub(super) observation: nuwax_nav_runtime::CostmapObservation,
    pub(super) map_frame: String,
}

#[cfg(feature = "guard-runtime")]
pub(super) fn commit_guarded_pose_telemetry(telemetry: &mut GuardedNavTelemetry, input: &GuardedPoseInput) {
    telemetry.map_frame = input.frame_id.clone();
    telemetry.pose_frame = input.frame_id.clone();
    telemetry.current = input.current;
    telemetry.current_z = input.pose.pose.position.z;
    update_progress(telemetry);
}

#[cfg(feature = "guard-runtime")]
pub(super) fn commit_guarded_costmap_telemetry(
    telemetry: &mut GuardedNavTelemetry,
    input: &GuardedCostmapInput,
) {
    telemetry.map_frame = input.map_frame.clone();
}

#[cfg(feature = "guard-runtime")]
pub(super) fn decode_guarded_costmap_input(payload: &[u8]) -> OwnerInputResult<GuardedCostmapInput> {
    let grid = match schema::decode_occupancy_grid_body(payload) {
        Ok(grid) => grid,
        Err(error) => {
            return Ok(OwnerInput::Rejected(format!(
                "OccupancyGrid decode: {error}"
            )));
        }
    };
    if grid.header.frame_id != "map" {
        return Ok(OwnerInput::Rejected(format!(
            "scheduler occupancy requires map frame, got frame={}",
            grid.header.frame_id,
        )));
    }
    if !grid.info.resolution.is_finite()
        || grid
            .info
            .origin
            .position
            .iter()
            .chain(grid.info.origin.orientation_xyzw.iter())
            .any(|value| !value.is_finite())
    {
        return Ok(OwnerInput::Rejected(
            "guarded_nav_scheduler.costmap metadata contains a non-finite value".into(),
        ));
    }
    let source_timestamp_ns =
        match lcm_header_timestamp_ns(&grid.header, "guarded_nav_scheduler.costmap") {
            Ok(timestamp) => timestamp,
            Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
        };
    let costmap = match decode_astar_costmap(payload) {
        Ok(costmap) => costmap,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    // 保留未知类别；全局规划拒绝未知区域，局部碰撞只判定真实占据障碍。
    Ok(OwnerInput::Accepted(GuardedCostmapInput {
        observation: nuwax_nav_runtime::CostmapObservation {
            costmap: nuwax_nav_runtime::ExecutionMap::Planar(costmap),
            source_timestamp_ns,
        },
        map_frame: grid.header.frame_id,
    }))
}

#[cfg(feature = "guard-runtime")]
pub(super) fn schema_pose_is_finite_and_normalized(pose: &schema::Pose) -> bool {
    let finite = pose
        .position
        .iter()
        .chain(pose.orientation_xyzw.iter())
        .all(|value| value.is_finite());
    let norm_squared = pose
        .orientation_xyzw
        .iter()
        .map(|value| value * value)
        .sum::<f64>();
    finite && norm_squared.is_finite() && norm_squared > f64::EPSILON
}

#[cfg(feature = "guard-runtime")]
pub(super) fn guarded_scheduler_result(
    scheduler_exited_first: bool,
    loop_result: NavStackResult<()>,
    shutdown_result: NavStackResult<()>,
    task_result: NavStackResult<()>,
) -> NavStackResult<()> {
    if scheduler_exited_first && task_result.is_err() {
        return task_result;
    }
    loop_result?;
    shutdown_result?;
    task_result
}

#[cfg(feature = "guard-runtime")]
pub(super) async fn coordinate_guarded_scheduler<LoopFuture, TaskFuture, Shutdown, ShutdownFuture>(
    loop_future: LoopFuture,
    task_future: TaskFuture,
    shutdown: Shutdown,
) -> NavStackResult<()>
where
    LoopFuture: Future<Output = NavStackResult<()>>,
    TaskFuture: Future<Output = NavStackResult<()>>,
    Shutdown: FnOnce() -> ShutdownFuture,
    ShutdownFuture: Future<Output = NavStackResult<()>>,
{
    tokio::pin!(loop_future);
    tokio::pin!(task_future);

    tokio::select! {
        biased;
        task_result = &mut task_future => {
            let loop_result = loop_future.await;
            guarded_scheduler_result(true, loop_result, Ok(()), task_result)
        }
        loop_result = &mut loop_future => {
            let shutdown_result = shutdown().await;
            let task_result = task_future.await;
            guarded_scheduler_result(false, loop_result, shutdown_result, task_result)
        }
    }
}

#[cfg(feature = "guard-runtime")]
pub(super) async fn run_guarded_nav_scheduler(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let source_max_age = configured_pose_source_max_age()?;
    let transport = lcm_transport()?;
    let goals = typed_stream(&transport, &node.inputs[0], "nuwax_msgs.NavigationGoal").await?;
    let poses = typed_stream(&transport, &node.inputs[1], "geometry_msgs.PoseStamped").await?;
    let paths = typed_stream(&transport, &node.inputs[2], "nuwax_msgs.PathEnvelope").await?;
    let costmap_type = if planar_navigation_enabled() {nuwax_codec::lcm::planar_navigation::TYPE_NAME} else {"nav_msgs.OccupancyGrid"};
    let costmaps = typed_stream(&transport, &node.inputs[3], costmap_type).await?;
    let cancels = typed_stream(&transport, &node.inputs[4], "nuwax_msgs.CancelNavigation").await?;
    let socket_path = nuwax_nav_runtime::motion_guard_socket_path();
    let scheduler_config = nuwax_nav_runtime::SchedulerConfig::default();
    let (scheduler, task) = nuwax_nav_runtime::spawn_production_scheduler(
        socket_path,
        scheduler_config,
    )
    .await
    .map_err(|error| module_error("guarded_nav_scheduler.connect", error))?;
    let status = scheduler.subscribe_status();
    signal_ready(ready)?;

    let loop_future = run_guarded_nav_loop(&node, &transport, &scheduler, shutdown, goals, poses, paths, costmaps, cancels, status, Some(source_max_age), planar_navigation_enabled(), &body_reference_frame_id, None, None);
    let task_future = async move {
        task.await
            .map_err(|error| NavStackError::Internal(format!("guard scheduler join: {error}")))?
            .map_err(|error| module_error("guarded_nav_scheduler.task", error))
    };
    coordinate_guarded_scheduler(loop_future, task_future, || async {
        scheduler
            .shutdown()
            .await
            .map_err(|error| module_error("guarded_nav_scheduler.shutdown", error))
    })
    .await
}

#[cfg(feature = "guard-runtime")]
pub(super) enum GuardedMapFrame {
    Planar(RawLcmFrame),
    #[cfg(feature = "mls")]
    Surface(surface_dataflow::PreparedSurfaceMap),
    #[cfg(feature = "mls")]
    RecentCloud(RawLcmFrame),
}

#[cfg(feature = "guard-runtime")]
impl From<RawLcmFrame> for GuardedMapFrame {
    fn from(frame: RawLcmFrame) -> Self {
        Self::Planar(frame)
    }
}

#[cfg(feature = "guard-runtime")]
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct GuardedMapAuthority {
    pub(super) session_id: String,
    pub(super) revision: i64,
    pub(super) frame_id: String,
}

#[cfg(feature = "guard-runtime")]
impl GuardedMapAuthority {
    pub(super) fn same_domain(&self, other: &Self) -> bool {
        self.session_id == other.session_id
    }

    pub(super) fn permits_goal(&self, goal: &schema::NavigationGoal) -> bool {
        self.frame_id == goal.goal.header.frame_id
    }
}

#[cfg(feature = "guard-runtime")]
pub(super) enum GuardedNavEvent {
    Pose(Option<TransportResult<RawLcmFrame>>),
    Costmap(Option<TransportResult<GuardedMapFrame>>),
    Status(Result<(), tokio::sync::watch::error::RecvError>),
    Goal(Option<TransportResult<RawLcmFrame>>),
    Path(Option<TransportResult<RawLcmFrame>>),
    Cancel(Option<TransportResult<RawLcmFrame>>),
}

#[cfg(feature = "guard-runtime")]
pub(super) async fn wait_guarded_nav_event<M, T>(
    goals: &mut LcmFrameStream,
    poses: &mut LcmFrameStream,
    paths: &mut LcmFrameStream,
    costmaps: &mut M,
    cancels: &mut LcmFrameStream,
    status: &mut tokio::sync::watch::Receiver<nuwax_nav_runtime::SchedulerStatus>,
) -> GuardedNavEvent
where
    M: Stream<Item = TransportResult<T>> + Unpin,
    T: Into<GuardedMapFrame>,
{
    tokio::select! {
        message = poses.next() => GuardedNavEvent::Pose(message),
        message = costmaps.next() => GuardedNavEvent::Costmap(message.map(|frame| frame.map(Into::into))),
        changed = status.changed() => GuardedNavEvent::Status(changed),
        message = goals.next() => GuardedNavEvent::Goal(message),
        message = paths.next() => GuardedNavEvent::Path(message),
        message = cancels.next() => GuardedNavEvent::Cancel(message),
    }
}

#[cfg(feature = "guard-runtime")]
#[allow(clippy::too_many_arguments)]
pub(super) async fn run_guarded_nav_loop<M, T>(
    node: &TaskChannels,
    transport: &LcmTransport,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    mut shutdown: oneshot::Receiver<()>,
    mut goals: LcmFrameStream,
    mut poses: LcmFrameStream,
    mut paths: LcmFrameStream,
    mut costmaps: M,
    mut cancels: LcmFrameStream,
    mut status: tokio::sync::watch::Receiver<nuwax_nav_runtime::SchedulerStatus>,
    source_max_age: Option<Duration>,
    planar_mode: bool,
    _body_reference_frame_id: &str,
    recent_source_max_age: Option<Duration>,
    lio_motion: Option<crate::map_pose::LioMotionReceiver>,
) -> NavStackResult<()>
where
    M: Stream<Item = TransportResult<T>> + Unpin + Send,
    T: Into<GuardedMapFrame> + Send,
{
    let process_runtime_generation = runtime_generation();
    let mut telemetry = GuardedNavTelemetry {
        runtime_generation: process_runtime_generation,
        surface_navigation: node.id == "surface_nav_scheduler",
        ..GuardedNavTelemetry::default()
    };
    let mut latest_goal: Option<schema::NavigationGoal> = None;
    let mut surface_authority: Option<GuardedMapAuthority> = None;
    #[cfg(feature="mls")]
    let mut recent_source=surface_dataflow::RecentSourceState::new(recent_source_max_age);
    if recent_source_max_age.is_some() {
        scheduler.update_obstacle_source(None,false).await
            .map_err(|error|module_error("recent source initial health",error))?;
    }
    let mut planar_admission=nuwax_codec::lcm::planar_navigation::Admission::default();
    let mut publications = GuardedStatusPublications::default();
    let mut diagnostics = InputDiagnosticLimiter::default();

    let initial_status = status.borrow().clone();
    publish_guarded_nav_observation(
        transport,
        node,
        initial_status,
        &telemetry,
        None,
        &mut publications,
    )
    .await?;

    loop {
        let event = tokio::select! {
            biased;
            _ = &mut shutdown => return Ok(()),
            event = wait_guarded_nav_event(
                &mut goals, &mut poses, &mut paths, &mut costmaps, &mut cancels, &mut status,
            ) => event,
        };
        match event {
            GuardedNavEvent::Status(changed) => {
                changed.map_err(|_| {
                    NavStackError::Internal("guard scheduler status stream closed".into())
                })?;
                let snapshot = status.borrow().clone();
                publish_guarded_tail_observation(transport, node, snapshot, &mut telemetry, None, &mut publications).await?;
            }
            GuardedNavEvent::Pose(message) => {
                let Some(input) = drain_guarded_pose_input(
                    message,
                    &mut poses,
                    &mut diagnostics,
                )?
                else {
                    continue;
                };
                let source_age_limit = source_max_age;
                let measured = lio_motion.as_ref().and_then(|source| {
                    source.borrow().as_ref()
                        .filter(|m| m.source_timestamp_ns == input.pose.stamp.nanos as u64)
                        .cloned()
                });
                let mut candidate_telemetry = telemetry.clone();
                commit_guarded_pose_telemetry(&mut candidate_telemetry, &input);
                let settlement = (async {
                    match source_age_limit {
                        Some(maximum) => scheduler
                            .update_odometry_with_lio_motion(input.pose, maximum, measured).await,
                        None => scheduler.update_odometry(input.pose).await,
                    }
                }).await;
                let accepted = settlement.is_ok();
                let ordinary = match settlement {
                    Ok(()) => { telemetry = candidate_telemetry; Ok(()) }
                    Err(error) => match runtime_input_error::<()>("guarded_nav_scheduler.map_pose", error) {
                        Ok(OwnerInput::Rejected(reason)) => {
                            diagnostics.reject("guarded_nav_scheduler.map_pose", &reason); Ok(())
                        }
                        Ok(OwnerInput::Accepted(())) => unreachable!("error classifier accepted an error"),
                        Err(error) => Err(error),
                    },
                };
                ordinary?;
                if !accepted { continue; }

                let snapshot = scheduler.status();
                let planner_inputs_fresh = snapshot.planner_inputs_fresh;
                publish_guarded_tail_observation(transport, node, snapshot, &mut telemetry, Some(planner_inputs_fresh), &mut publications).await?;
            }
            GuardedNavEvent::Costmap(message) => {
                let frame = message
                    .ok_or_else(|| NavStackError::Internal("guarded map stream closed".into()))?
                    .map_err(|error| module_error("guarded map input", error))?;
                let (decoded, received_at) = match frame {
                    #[cfg(feature="mls")]
                    GuardedMapFrame::RecentCloud(frame) => {
                        let settlement = (recent_source.consume(&frame.payload, surface_authority.as_ref(), scheduler)).await;
                        if let Err(reason) = settlement {
                            diagnostics.reject("recent obstacle source", &reason);
                        }

                        continue;
                    }
                    GuardedMapFrame::Planar(frame) => {
                        let source_received_at = None;
                        let body = if planar_mode {
                            let value=match nuwax_codec::lcm::planar_navigation::decode_body(&frame.payload) {
                                Ok(value)=>value,
                                Err(error)=>{diagnostics.reject("planar_navigation",&error.to_string());continue;}
                            };
                            let changed=match planar_admission.admit(&value) {
                                Ok(changed)=>changed,
                                Err(error)=>{diagnostics.reject("planar_navigation",&error);continue;}
                            };

                            let authority = GuardedMapAuthority {
                                session_id: value.session_id.clone(),
                                revision: value.map_revision,
                                frame_id: value.grid.header.frame_id.clone(),
                            };
                            if changed || value.cleared {
                                // 清空与切域由会话和观测修订隔离，不以时间戳建立水位。
                                scheduler.invalidate_map_source("二维导航来源已撤销或切换",
                                    nuwax_nav_runtime::MapInvalidationKind::Invalidated).await
                                    .map_err(|e|module_error("planar map invalidation",e))?;

                                let snapshot=scheduler.status();
                                publish_guarded_nav_observation(transport,node,snapshot.clone(),&telemetry,
                                    Some(false),&mut publications).await?;
                            }
                            surface_authority=if value.cleared {None} else {Some(authority)};
                            if value.cleared {continue;}
                            schema::encode_occupancy_grid_body(&value.grid)
                                .map_err(|e|NavStackError::Internal(e.to_string()))?
                        } else {frame.payload.to_vec()};
                        (decode_guarded_costmap_input(&body)?, source_received_at)
                    }
                    #[cfg(feature = "mls")]
                    GuardedMapFrame::Surface(update) => {
                        let changed_domain = surface_authority
                            .as_ref()
                            .is_some_and(|previous| !previous.same_domain(&update.authority));
                        if changed_domain || update.snapshot.is_err() {
                            scheduler
                                .invalidate_map_source(
                                    update
                                        .snapshot
                                        .as_ref()
                                        .err()
                                        .cloned()
                                        .unwrap_or_else(|| "surface map domain changed".into()),
                                    if changed_domain || !update.awaiting_build {
                                        nuwax_nav_runtime::MapInvalidationKind::Invalidated
                                    } else {
                                        nuwax_nav_runtime::MapInvalidationKind::TemporaryUnavailable
                                    },
                                )
                                .await
                                .map_err(|error| module_error("surface map invalidation", error))?;

                            // 精细源失败或换域时保留待执行目标，之后的取消仍须
                            // 通过原状态通道回报该目标。

                            let snapshot = scheduler.status();
                            publish_guarded_nav_observation(
                                transport,
                                node,
                                snapshot.clone(),
                                &telemetry,
                                Some(false),
                                &mut publications,
                            )
                            .await?;
                        }
                        if changed_domain || update.snapshot.is_err() {
                            let settlement = (recent_source.reset(scheduler)).await;
                            let ordinary = settlement.map_err(|error|module_error("recent source domain reset",error));
                            ordinary?;
                        }
                        surface_authority = Some(update.authority.clone());
                        let surface = match update.snapshot {
                            Ok(surface) => surface,
                            Err(reason) => {
                                diagnostics.reject("surface map", &reason);
                                continue;
                            }
                        };
                        let surface=recent_source.attach_terrain(surface);
                        (
                            OwnerInput::Accepted(GuardedCostmapInput {
                                map_frame: update.authority.frame_id.clone(),
                                observation: nuwax_nav_runtime::CostmapObservation {
                                    costmap: nuwax_nav_runtime::ExecutionMap::SurfaceUpdate(nuwax_nav_runtime::surface_changes::SurfaceMapUpdate {
                                        snapshot: surface, change_basis: update.change_basis, changes: update.geometry_changes,
                                    }),
                                    source_timestamp_ns: update.source_timestamp_ns,
                                },
                            }),
                            Some(update.received_at),
                        )
                    }
                };
                let input = match decoded {
                    OwnerInput::Accepted(input) => input,
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("guarded_nav_scheduler.costmap", &reason);
                        continue;
                    }
                };
                // 保留活动会话；只有显式替换目标才能抢占。
                let mut candidate_telemetry = telemetry.clone();
                commit_guarded_costmap_telemetry(&mut candidate_telemetry, &input);
                let settlement = (scheduler.update_costmap_received(input.observation,
                        tokio::time::Instant::from_std(received_at.unwrap_or_else(Instant::now)))).await;
                let accepted = settlement.is_ok();
                let ordinary = match settlement {
                    Ok(()) => {
                        telemetry = candidate_telemetry;
                        // Observe the accepted map without taking/activating a
                        // buffered goal before an early cancellation settles.

                        Ok(())
                    }
                    Err(error) => match runtime_input_error::<()>("guarded_nav_scheduler.costmap", error) {
                        Ok(OwnerInput::Rejected(reason)) => {
                            diagnostics.reject("guarded_nav_scheduler.costmap", &reason); Ok(())
                        }
                        Ok(OwnerInput::Accepted(())) => unreachable!("error classifier accepted an error"),
                        Err(error) => Err(error),
                    },
                };
                ordinary?;
                if !accepted { continue; }

                let snapshot = scheduler.status();
                let planner_inputs_fresh = snapshot.planner_inputs_fresh;
                publish_guarded_tail_observation(transport, node, snapshot, &mut telemetry, Some(planner_inputs_fresh), &mut publications).await?;
            }
            GuardedNavEvent::Goal(message) => {
                let frame = required_frame(message, "guarded_nav_scheduler.goal")?;
                let navigation_goal = match decode_navigation_goal_input(&frame.payload) {
                    Ok(envelope) => envelope,
                    Err(error) => {
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &telemetry,
                            "goal",
                            &format!("NavigationGoal rejected: {error}"),
                            &mut publications,
                        )
                        .await?;
                        continue;
                    }
                };
                if latest_goal.as_ref()==Some(&navigation_goal){continue;}
                activate_guarded_goal(
                    node,
                    transport,
                    scheduler,
                    &mut telemetry,
                    &mut latest_goal,
                    navigation_goal,
                    &mut publications,
                )
                .await?;
            }
            GuardedNavEvent::Path(message) => {
                let frame = required_frame(message, "guarded_nav_scheduler.path")?;
                let Some(active_goal) = latest_goal.as_ref() else {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "path",
                        "no active versioned goal lease",
                        &mut publications,
                    )
                    .await?;
                    continue;
                };

                let path_envelope = match schema::decode_path_envelope_body(&frame.payload) {
                    Ok(envelope) => envelope,
                    Err(error) => {
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &telemetry,
                            "path",
                            &format!("PathEnvelope rejected: {error}"),
                            &mut publications,
                        )
                        .await?;
                        continue;
                    }
                };
                if let Err(reason) = validate_path_goal_correlation(&path_envelope, active_goal) {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "path",
                        &reason,
                        &mut publications,
                    )
                    .await?;
                    continue;
                }
                let path = match execution_path_from_schema(
                    &path_envelope.path,
                    node.id == "surface_nav_scheduler",
                ) {
                    Ok(path) => path,
                    Err(error) => {
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &telemetry,
                            "path",
                            &error.to_string(),
                            &mut publications,
                        )
                        .await?;
                        continue;
                    }
                };
                let settlement = (scheduler
                    .submit_path(nuwax_nav_runtime::VersionedPath {
                        route_kind: if path_envelope.route_kind == 1 {nuwax_nav_runtime::RouteKind::Approach}
                            else {nuwax_nav_runtime::RouteKind::Complete},
                        session_id: path_envelope.session_id.clone(),
                        execution_identity: guarded_path_execution_identity(&path_envelope),
                        path_revision: u64::try_from(path_envelope.path_revision)
                            .expect("validated positive path revision"),
                        path_yaws: if node.id == "surface_nav_scheduler" {
                            None
                        } else {
                            Some(
                                path_envelope
                                    .path
                                    .poses
                                    .iter()
                                    .map(|pose| {
                                        let [x, y, z, w] = pose.pose.orientation_xyzw;
                                        Quaternion { x, y, z, w }.yaw()
                                    })
                                    .collect(),
                            )
                        },
                        path,
                    })).await;
                let ordinary = match settlement {
                    Err(error) => match runtime_input_error::<()>("guarded_nav_scheduler.path", error) {
                        Ok(OwnerInput::Rejected(reason)) => publish_guarded_rejection(
                            transport, &node.outputs[1], scheduler.status(), &telemetry, "path", &reason,
                            &mut publications,
                        ).await,
                        Ok(OwnerInput::Accepted(())) => unreachable!("error classifier accepted an error"),
                        Err(error) => Err(error),
                    },
                    Ok(()) => {
                        {
                            telemetry.path_revision = path_envelope.path_revision;
                            telemetry.planner_runtime_generation = path_envelope.planner_runtime_generation;
                        }
                        Ok(())
                    }
                };
                ordinary?;
            }
            GuardedNavEvent::Cancel(message) => {
                let frame = required_frame(message, "导航取消")?;
                let cancel = match schema::decode_cancel_navigation_body(&frame.payload) {
                    Ok(cancel) => cancel,
                    Err(error) => { diagnostics.reject("导航取消", &error.to_string()); continue; }
                };
                if latest_goal.as_ref().is_some_and(|goal| validate_cancel_goal_correlation(&cancel, goal).is_ok()) {
                    if let Err(error) = scheduler.cancel(&cancel.session_id).await {
                        if let OwnerInput::Rejected(reason) = runtime_input_error::<()>("导航取消", error)? {
                            diagnostics.reject("导航取消", &reason);
                        }
                    }
                    let snapshot = scheduler.status();
                    publish_guarded_nav_observation(transport, node, snapshot.clone(), &telemetry, None, &mut publications).await?;
                }

            }
        }
    }
}

#[cfg(feature = "guard-runtime")]
pub(super) async fn activate_guarded_goal(
    node: &TaskChannels,
    transport: &LcmTransport,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    telemetry: &mut GuardedNavTelemetry,
    latest_goal: &mut Option<schema::NavigationGoal>,
    navigation_goal: schema::NavigationGoal,
    publications: &mut GuardedStatusPublications,
) -> NavStackResult<()> {
    let goal = pose_from_schema(&navigation_goal.goal)?;
    // 容差等选项变化也代表替换目标，本次状态立即发布。
    publications.state_published_at = None;
    telemetry.goal = [
        goal.pose.position.x,
        goal.pose.position.y,
        goal.pose.orientation.yaw(),
    ];
    telemetry.goal_z = goal.pose.position.z;
    telemetry.public_session_id = navigation_goal.session_id.clone();
    telemetry.motion_generation = navigation_goal.motion_generation as i64;
    telemetry.path_revision = 0;
    telemetry.planner_runtime_generation = 0;
    telemetry.initial_distance = telemetry_distance(telemetry);
    telemetry.progress = 0.;
    *latest_goal = Some(navigation_goal.clone());
    let result = scheduler
        .start_session(nuwax_nav_runtime::NavSession {
            boundary: navigation_goal
                .boundary_points
                .iter()
                .map(|p| [p[0], p[1]])
                .collect(),
            options: navigation_goal.options(),
            goal: nuwax_nav_runtime::ExecutionGoal::Body(goal),
            session_id: navigation_goal.session_id.clone(),
            execution_identity: guarded_goal_execution_identity(&navigation_goal),
            motion_generation: navigation_goal.motion_generation,
        })
        .await;
    if let Err(error) = result {
        match runtime_input_error::<()>("guarded_nav_scheduler.goal", error)? {
            OwnerInput::Rejected(reason) => {
                publish_guarded_rejection(
                    transport,
                    &node.outputs[1],
                    scheduler.status(),
                    telemetry,
                    "goal",
                    &reason,
                    publications,
                )
                .await?
            }
            OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
        }
    } else {
        publish_guarded_nav_state(
            transport,
            &node.outputs[1],
            scheduler.status(),
            telemetry,
            None,
            publications,
        )
        .await?;
    }
    Ok(())
}

#[cfg(feature = "guard-runtime")]
pub(super) fn runtime_generation() -> u64 {
    static PROCESS_RUNTIME_GENERATION: std::sync::OnceLock<u64> = std::sync::OnceLock::new();
    *PROCESS_RUNTIME_GENERATION.get_or_init(|| {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos()
            .min(u64::MAX as u128) as u64
    })
}

#[cfg(feature = "guard-runtime")]
pub(super) fn clear_guarded_session_telemetry(telemetry: &mut GuardedNavTelemetry) {
    telemetry.public_session_id.clear();
    telemetry.motion_generation = 0;
    telemetry.goal = [0.0; 3];
    telemetry.goal_z = 0.0;
    telemetry.path_revision = 0;
    telemetry.planner_runtime_generation = 0;
    telemetry.initial_distance = 0.0;
    telemetry.progress = 0.0;
}

#[cfg(feature = "guard-runtime")]
pub(super) fn validate_path_goal_correlation(path: &schema::PathEnvelope, goal: &schema::NavigationGoal) -> Result<(), String> {
    if path.session_id != goal.session_id {
        return Err("PathEnvelope goal identity does not match the active NavigationGoal".into());
    }
    Ok(())
}

#[cfg(feature = "guard-runtime")]
pub(super) fn validate_cancel_goal_correlation(
    cancel: &schema::CancelNavigation,
    goal: &schema::NavigationGoal,
) -> Result<(), String> {
    if cancel.session_id.is_empty() || cancel.session_id == goal.session_id { Ok(()) }
    else { Err("取消目标号不匹配".into()) }
}

#[cfg(any(feature = "guard-runtime", feature = "click-goal"))]
pub(super) fn lcm_header_timestamp_ns(header: &schema::Header, component: &str) -> NavStackResult<i64> {
    if header.stamp_sec < 0 || !(0..1_000_000_000).contains(&header.stamp_nsec) {
        return Err(NavStackError::InvalidConfig(format!(
            "{component} has an invalid source timestamp",
        )));
    }
    i64::from(header.stamp_sec)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(i64::from(header.stamp_nsec)))
        .filter(|timestamp| *timestamp > 0)
        .ok_or_else(|| {
            NavStackError::InvalidConfig(format!("{component} source timestamp must be positive",))
        })
}

#[cfg(feature = "guard-runtime")]
pub(super) fn telemetry_distance(telemetry: &GuardedNavTelemetry) -> f64 {
    let xy = distance_2d(telemetry.goal, telemetry.current);
    if telemetry.surface_navigation {
        xy.hypot(telemetry.goal_z - telemetry.current_z)
    } else {
        xy
    }
}

#[cfg(feature = "guard-runtime")]
pub(super) fn update_progress(telemetry: &mut GuardedNavTelemetry) {
    if telemetry.initial_distance > f64::EPSILON {
        let remaining = telemetry_distance(telemetry);
        telemetry.progress = (1.0 - remaining / telemetry.initial_distance).clamp(0.0, 1.0);
    }
}

#[cfg(feature = "guard-runtime")]
#[derive(Default)]
pub(super) struct GuardedStatusPublications {
    pub(super) goal: Option<(String, i64)>,
    pub(super) applied_commands: u64,
    pub(super) last_terminal: Option<nuwax_nav_runtime::TerminalOutcome>,
    pub(super) reached_published: bool,
    pub(super) state_goal: Option<(String, i64, [f64; 3], f64)>,
    pub(super) state_phase: Option<nuwax_nav_runtime::SchedulerPhase>,
    pub(super) state_terminal: Option<nuwax_nav_runtime::TerminalOutcome>,
    pub(super) state_published_at: Option<Instant>,
}

#[cfg(feature = "guard-runtime")]
impl GuardedStatusPublications {
    pub(super) fn frames(
        &mut self,
        node: &TaskChannels,
        status: &nuwax_nav_runtime::SchedulerStatus,
        telemetry: &GuardedNavTelemetry,
        planner_observation: Option<bool>,
    ) -> NavStackResult<Vec<RawLcmFrame>> {
        let mut frames = Vec::with_capacity(4);
        if !telemetry.public_session_id.is_empty() {
            let goal = (
                telemetry.public_session_id.clone(),
                telemetry.motion_generation,
            );
            if self.goal.as_ref() != Some(&goal) {
                self.goal = Some(goal);
                self.applied_commands = 0;
                self.last_terminal = None;
                self.reached_published = false;
            }
            if status.applied_commands > self.applied_commands {
                if let Some(twist) = status.last_applied_twist {
                    frames.push(build_twist2_frame(&node.outputs[0], twist)?);
                }
                self.applied_commands = status.applied_commands;
            }
            if status.last_terminal.is_some() && status.last_terminal != self.last_terminal {
                frames.push(build_twist2_frame(&node.outputs[0], Twist2::ZERO)?);
            }
            self.last_terminal = status.last_terminal;
            if status.last_terminal == Some(nuwax_nav_runtime::TerminalOutcome::Succeeded)
                && !self.reached_published
            {
                frames.push(build_body_frame(
                    &node.outputs[2],
                    "std_msgs.Bool",
                    schema::encode_bool_body(&schema::Bool { data: true }),
                )?);
                self.reached_published = true;
            }
        }
        if let Some(frame) = self.state_frame_at(&node.outputs[1], status, telemetry, planner_observation, Instant::now())? {
            frames.push(frame);
        }
        Ok(frames)
    }
    pub(super) fn state_frame_at(
        &mut self, channel: &str, status: &nuwax_nav_runtime::SchedulerStatus,
        telemetry: &GuardedNavTelemetry, planner_observation: Option<bool>, now: Instant,
    ) -> NavStackResult<Option<RawLcmFrame>> {
        let goal = (telemetry.public_session_id.clone(), telemetry.motion_generation, telemetry.goal, telemetry.goal_z);
        let immediate = self.state_goal.as_ref() != Some(&goal)
            || self.state_phase != Some(status.phase)
            || self.state_terminal != status.last_terminal;
        if !immediate && self.state_published_at.is_some_and(|last| now.duration_since(last) < Duration::from_millis(100)) {
            return Ok(None);
        }
        let frame = build_guarded_nav_state_frame(channel, status, telemetry, planner_observation)?;
        self.state_goal = Some(goal);
        self.state_phase = Some(status.phase);
        self.state_terminal = status.last_terminal;
        self.state_published_at = Some(now);
        Ok(Some(frame))
    }

}

#[cfg(feature = "guard-runtime")]
pub(super) async fn publish_guarded_tail_observation(
    transport: &LcmTransport, node: &TaskChannels, snapshot: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &mut GuardedNavTelemetry, planner_observation: Option<bool>,
    publications: &mut GuardedStatusPublications,
) -> NavStackResult<()> {
    publish_guarded_nav_observation(transport, node, snapshot.clone(), telemetry, planner_observation, publications).await?;
    clear_terminal_telemetry_after_publication(telemetry, &snapshot);
    Ok(())
}

#[cfg(feature = "guard-runtime")]
pub(super) fn clear_terminal_telemetry_after_publication(
    telemetry: &mut GuardedNavTelemetry,
    status: &nuwax_nav_runtime::SchedulerStatus,
) {
    if status.last_terminal.is_some() {
        clear_guarded_session_telemetry(telemetry);
    }
}

#[cfg(feature = "guard-runtime")]
pub(super) async fn publish_guarded_nav_observation(
    transport: &LcmTransport,
    node: &TaskChannels,
    status: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
    publications: &mut GuardedStatusPublications,
) -> NavStackResult<()> {
    let frames=publications.frames(node, &status, telemetry, planner_observation)?;

    transport
        .publish_batch(frames)
        .await
        .map_err(|error| NavStackError::Internal(format!("publish guard status batch: {error}")))?;

    Ok(())
}

#[cfg(feature = "guard-runtime")]
pub(super) async fn publish_guarded_nav_state(
    transport: &LcmTransport,
    channel: &str,
    status: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
    publications: &mut GuardedStatusPublications,
) -> NavStackResult<()> {
    if let Some(frame) = publications.state_frame_at(channel, &status, telemetry, planner_observation, Instant::now())? {
        transport.publish(frame).await
            .map_err(|error| NavStackError::Internal(format!("publish {channel}: {error}")))?;
    }
    Ok(())
}

#[cfg(feature = "guard-runtime")]
pub(super) fn build_guarded_nav_state_frame(
    channel: &str,
    status: &nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
) -> NavStackResult<RawLcmFrame> {
    let message = build_guarded_navigation_status(
        &status,
        telemetry,
        planner_observation,
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default(),
    );
    let body = schema::encode_navigation_status_body(&message)
        .map_err(|error| NavStackError::Internal(format!("NavigationStatus encode: {error}")))?;
    build_body_frame(channel, "nuwax_msgs.NavigationStatus", body)
}

#[cfg(feature = "guard-runtime")]
pub(super) fn build_guarded_navigation_status(
    status: &nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
    now: Duration,
) -> schema::NavigationStatus {
    use nuwax_nav_runtime::{SchedulerPhase, TerminalOutcome};

    let has_public_session = !telemetry.public_session_id.is_empty();
    let state = if !has_public_session {
        0
    } else {
        match (status.phase, status.last_terminal) {
            (_, Some(TerminalOutcome::Stopped)) => 4,
            (_, Some(TerminalOutcome::Succeeded)) => 3,
            (_, Some(TerminalOutcome::Cancelled | TerminalOutcome::Shutdown)) => 0,
            (SchedulerPhase::Failed, _) => 4,
            (SchedulerPhase::Controlling, _) => 2,
            (SchedulerPhase::Acquiring | SchedulerPhase::AwaitingPath, _) => 1,
            _ => 0,
        }
    };
    let goal_status = if !has_public_session {
        schema::NAVIGATION_GOAL_STATUS_NONE
    } else {
        match (status.phase, status.last_terminal) {
            (_, Some(TerminalOutcome::Stopped)) => schema::NAVIGATION_GOAL_STATUS_STOPPED,
            (_, Some(TerminalOutcome::Succeeded)) => schema::NAVIGATION_GOAL_STATUS_SUCCEEDED,
            (_, Some(TerminalOutcome::Cancelled | TerminalOutcome::Shutdown)) => schema::NAVIGATION_GOAL_STATUS_CANCELLED,
            (SchedulerPhase::Failed, _) => schema::NAVIGATION_GOAL_STATUS_FAILED,
            (SchedulerPhase::Controlling, _) => schema::NAVIGATION_GOAL_STATUS_EXECUTING,
            (SchedulerPhase::Acquiring | SchedulerPhase::AwaitingPath, _) => schema::NAVIGATION_GOAL_STATUS_PLANNING,
            _ => schema::NAVIGATION_GOAL_STATUS_NONE,
        }
    };
    let input_ready = status.motion_sink_ready && status.planner_inputs_fresh
        && !telemetry.map_frame.is_empty() && telemetry.map_frame == telemetry.pose_frame;
    let message = if !input_ready && !matches!(goal_status, schema::NAVIGATION_GOAL_STATUS_SUCCEEDED | schema::NAVIGATION_GOAL_STATUS_FAILED | schema::NAVIGATION_GOAL_STATUS_CANCELLED | schema::NAVIGATION_GOAL_STATUS_STOPPED) {
        let source_now = nuwax_nav_types_messages::clock::source_now_ns();
        let age = |stamp: i64| {
            if stamp <= 0 { return "尚未收到，年龄未知".to_string(); }
            let stamp = stamp as u64;
            if stamp > source_now.saturating_add(nuwax_nav_types_messages::clock::SOURCE_FUTURE_TOLERANCE_NS) {
                "来源年龄不可用：数据源时间超前当前来源时钟超过500毫秒".to_string()
            } else {
                format!("来源年龄={}毫秒", source_now.saturating_sub(stamp) / 1_000_000)
            }
        };
        let mut missing = Vec::new();
        if !status.motion_sink_ready { missing.push("运动守卫未就绪".to_string()); }
        if !status.planner_inputs_fresh {
            missing.push(format!("{}；位姿{}，接收年龄={}毫秒；栅格/曲面{}，接收年龄={}毫秒",
                status.input_wait_reason.as_deref().unwrap_or("规划输入尚未就绪"),
                age(status.pose_source_timestamp_ns), status.pose_receipt_age_ms,
                age(status.costmap_source_timestamp_ns), status.costmap_receipt_age_ms));
        }
        if telemetry.map_frame.is_empty() || telemetry.map_frame != telemetry.pose_frame {
            missing.push(format!("地图/位姿坐标系未就绪：地图={}，位姿={}", telemetry.map_frame, telemetry.pose_frame));
        }
        format!("等待导航输入：{}", missing.join("；"))
    } else if has_public_session { status.message.clone() } else { "导航栈空闲".into() };
    let include_planner_fields = planner_observation == Some(true);
    schema::NavigationStatus {
        session_id: telemetry.public_session_id.clone(),
        robot_frame: "base_link".into(),
        stamp_sec: now.as_secs().min(i64::MAX as u64) as i64,
        stamp_nsec: now.subsec_nanos() as i32,
        state,
        goal_status,
        reason: message.clone(),
        remaining_distance_m: status.remaining_distance_m,
        input_ready,
        no_feasible_path: status.no_feasible_path && goal_status == schema::NAVIGATION_GOAL_STATUS_PLANNING,
        goal: telemetry.goal,
        current: telemetry.current,
        progress: telemetry.progress,
        message,
        runtime_generation: saturating_i64(telemetry.runtime_generation),

        pose_source_timestamp_ns: status.pose_source_timestamp_ns,
        costmap_source_timestamp_ns: status.costmap_source_timestamp_ns,
        sink_ready: i8::from(status.motion_sink_ready),
        planner_map_frame: if include_planner_fields {
            telemetry.map_frame.clone()
        } else {
            String::new()
        },
        planner_pose_frame: if include_planner_fields {
            telemetry.pose_frame.clone()
        } else {
            String::new()
        },
        motion_generation: telemetry.motion_generation,
        path_revision: telemetry.path_revision,
        planner_runtime_generation: telemetry.planner_runtime_generation,
        local_path: if state == 2
            && status.session_id.as_deref() == Some(telemetry.public_session_id.as_str())
            && i64::try_from(status.path_revision).ok() == Some(telemetry.path_revision)
        { status.local_path.clone() } else { Vec::new() },
    }
}

#[cfg(feature = "guard-runtime")]
pub(super) async fn publish_guarded_rejection(
    transport: &LcmTransport,
    channel: &str,
    status: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    input: &str,
    reason: &str,
    publications: &mut GuardedStatusPublications,
) -> NavStackResult<()> {
    tracing::warn!(input, reason, session_id = %telemetry.public_session_id, "导航输入被拒绝");
    let status = guarded_rejection_status(status, input, reason);
    publish_guarded_nav_state(transport, channel, status, telemetry, None, publications).await
}

#[cfg(feature = "guard-runtime")]
pub(super) fn guarded_rejection_status(
    mut status: nuwax_nav_runtime::SchedulerStatus,
    input: &str,
    reason: &str,
) -> nuwax_nav_runtime::SchedulerStatus {
    // 停车中的原因和已确认终态不能被迟到路径的协议拒绝覆盖。
    if input=="path" && (status.last_terminal.is_some() || matches!(status.phase,
        nuwax_nav_runtime::SchedulerPhase::Cancelling|nuwax_nav_runtime::SchedulerPhase::Failed)) {
        tracing::debug!(reason,session_id=?status.session_id,"迟到路径被拒绝，保留原停车或终态原因");
        return status;
    }
    status.message = format!("REJECTED {input}: {reason}");
    status
}

#[cfg(feature = "guard-runtime")]
pub(super) fn saturating_i64(value: u64) -> i64 {
    value.min(i64::MAX as u64) as i64
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime"
))]
pub(super) fn distance_2d(left: [f64; 3], right: [f64; 3]) -> f64 {
    (left[0] - right[0]).hypot(left[1] - right[1])
}

#[cfg(all(test, feature = "guard-runtime"))]
#[test]
fn navigation_status_reports_one_input_ready_signal_and_typed_goal_reason() {
    let mut status = nuwax_nav_runtime::SchedulerStatus {
        phase: nuwax_nav_runtime::SchedulerPhase::AwaitingPath,
        motion_sink_ready: true, planner_inputs_fresh: true,
        message: "正在等待可通行路线".into(), no_feasible_path: true,
        remaining_distance_m: 8.0, ..Default::default()
    };
    let telemetry = GuardedNavTelemetry { public_session_id: "目标".into(), map_frame: "map".into(), pose_frame: "map".into(), ..Default::default() };
    let ready = build_guarded_navigation_status(&status, &telemetry, None, Duration::ZERO);
    assert!(ready.input_ready);
    assert_eq!(ready.goal_status, schema::NAVIGATION_GOAL_STATUS_PLANNING);
    assert_eq!(ready.reason, "正在等待可通行路线");
    assert!(ready.no_feasible_path);
    assert_eq!(ready.remaining_distance_m, 8.0);
    status.planner_inputs_fresh = false;
    status.input_wait_reason = Some("recent obstacle source revoked".into());
    let waiting = build_guarded_navigation_status(&status, &telemetry, None, Duration::ZERO);
    assert!(!waiting.input_ready);
    assert!(waiting.reason.contains("recent obstacle source revoked"));
    assert!(waiting.reason.contains("尚未收到，年龄未知"));
    status.last_terminal = Some(nuwax_nav_runtime::TerminalOutcome::Stopped);
    assert_eq!(build_guarded_navigation_status(&status, &telemetry, None, Duration::ZERO).goal_status, schema::NAVIGATION_GOAL_STATUS_STOPPED);
    assert_eq!(build_guarded_navigation_status(&status, &telemetry, None, Duration::ZERO).reason, status.message);
}

#[cfg(all(test, feature = "guard-runtime"))]
#[test]
fn go2_input_status_reports_ages_in_the_shared_source_clock() {
    const CHILD: &str = "NEH_TEST_GO2_GUARDED_OFFSET";
    const OFFSET: u64 = 71_844_000_000_000;
    if std::env::var_os(CHILD).is_none() {
        let path = std::env::temp_dir().join(format!("neh-go2-guarded-offset-{}", std::process::id()));
        std::fs::write(&path, (OFFSET as i64).to_ne_bytes()).unwrap();
        let name = format!("{}::go2_input_status_reports_ages_in_the_shared_source_clock",
            module_path!().split_once("::").unwrap().1);
        let status = std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", &name]).env(CHILD, "1").env("NUWAX_DEVICE_CLOCK_OFFSET", &path)
            .status().unwrap();
        std::fs::remove_file(path).unwrap();
        assert!(status.success());
        return;
    }
    let source = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
        .unwrap().as_nanos() as u64 + OFFSET;
    let status = nuwax_nav_runtime::SchedulerStatus {
        pose_source_timestamp_ns: (source + 400_000_000) as i64,
        costmap_source_timestamp_ns: (source + 900_000_000) as i64,
        ..Default::default()
    };
    let message = build_guarded_navigation_status(&status, &GuardedNavTelemetry::default(), None, Duration::ZERO).reason;
    assert!(message.contains("位姿来源年龄=0毫秒"), "{message}");
    assert!(message.contains("栅格/曲面来源年龄不可用：数据源时间超前当前来源时钟超过500毫秒"), "{message}");
}
