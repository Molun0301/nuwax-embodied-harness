//! 二维规划：规划器输入、简单规划器、A* 规划器与基础路径跟随。

use super::*;

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
pub(super) fn decode_planner_pose_input(payload: &[u8], component: &str) -> OwnerInputResult<PoseStamped> {
    let pose_message =
        match schema::decode_pose_stamped(&prefixed(schema::POSE_STAMPED_FINGERPRINT, payload)) {
            Ok(message) => message,
            Err(error) => {
                return Ok(OwnerInput::Rejected(format!(
                    "{component} PoseStamped decode: {error}"
                )));
            }
        };
    if pose_message.header.frame_id != "map" {
        return Ok(OwnerInput::Rejected(format!(
            "{component} frame must be map, got {}",
            pose_message.header.frame_id,
        )));
    }
    if let Err(error) = lcm_header_timestamp_ns(&pose_message.header, component) {
        return Ok(OwnerInput::Rejected(error.to_string()));
    }
    if !schema_pose_is_finite_and_normalized(&pose_message.pose) {
        return Ok(OwnerInput::Rejected(format!(
            "{component} contains non-finite values or a zero-norm quaternion"
        )));
    }
    match pose_from_schema(&pose_message) {
        Ok(pose) => Ok(OwnerInput::Accepted(pose)),
        Err(error) => Ok(OwnerInput::Rejected(error.to_string())),
    }
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
pub(super) fn decode_planner_grid_input(
    payload: &[u8],
    component: &str,
) -> OwnerInputResult<schema::OccupancyGrid> {
    let grid = match schema::decode_occupancy_grid_body(payload) {
        Ok(grid) => grid,
        Err(error) => {
            return Ok(OwnerInput::Rejected(format!(
                "{component} OccupancyGrid decode: {error}"
            )));
        }
    };
    if grid.header.frame_id != "map" {
        return Ok(OwnerInput::Rejected(format!(
            "{component} requires map frame, got frame={}",
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
        return Ok(OwnerInput::Rejected(format!(
            "{component} metadata contains a non-finite value"
        )));
    }
    if let Err(error) = lcm_header_timestamp_ns(&grid.header, component) {
        return Ok(OwnerInput::Rejected(error.to_string()));
    }
    Ok(OwnerInput::Accepted(grid))
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
pub(super) fn decode_planner_stop_input(payload: &[u8], component: &str) -> OwnerInputResult<bool> {
    match schema::decode_bool_body(payload) {
        Ok(stop) => Ok(OwnerInput::Accepted(stop.data)),
        Err(error) => Ok(OwnerInput::Rejected(format!(
            "{component} Bool decode: {error}"
        ))),
    }
}



#[cfg(feature = "simple-2d")]
pub(super) fn apply_simple_planner_goal(
    module: &mut nuwax_nav_simple_planner::module::SimplePlannerModule,
    envelope: schema::NavigationGoal,
    occupancy_body: &[u8],
) -> OwnerInputResult<(schema::NavigationGoal, Option<NavigationBoundary>)> {
    use nuwax_nav_simple_planner::module::Module;

    let boundary = match navigation_boundary_from_goal(&envelope) {
        Ok(boundary) => boundary,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let pose = match pose_from_schema(&envelope.goal) {
        Ok(pose) => pose,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let occupancy = match boundary.as_ref() {
        Some(boundary) => {
            match decode_simple_occupancy_with_boundary(occupancy_body, Some(boundary)) {
                Ok(occupancy) => occupancy,
                Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
            }
        }
        None => match decode_simple_occupancy(occupancy_body) {
            Ok(occupancy) => occupancy,
            Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
        },
    };
    module
        .on_occupancy_grid(occupancy)
        .map_err(|error| module_error("simple_planner.goal_boundary", error))?;
    if let Err(error) = Module::on_goal(
        module,
        PointStamped::new(pose.frame_id, pose.stamp, pose.pose.position),
    ) {
        return module_input_error("simple_planner.goal_pose", error);
    }
    module.set_goal_yaw(pose.pose.orientation.yaw());
    // 目标身份保留原信封，自主导航只收窄其允许的运动方向。
    module.set_goal_options(nuwax_nav_types_messages::GoalOptions {allow_reverse:false,..envelope.options()})
        .map_err(|error|module_error("simple_planner.goal_options",error))?;
    Ok(OwnerInput::Accepted((envelope, boundary)))
}

#[cfg(feature = "replanning-astar")]
pub(super) fn apply_astar_planner_goal(
    module: &mut nuwax_nav_astar::module::ReplanningAStarPlannerModule,
    envelope: schema::NavigationGoal,
    costmap_body: &[u8],
) -> OwnerInputResult<(schema::NavigationGoal, Option<NavigationBoundary>, f64)> {
    use nuwax_nav_astar::module::Module;

    let boundary = match navigation_boundary_from_goal(&envelope) {
        Ok(boundary) => boundary,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let goal_pose = match pose_from_schema(&envelope.goal) {
        Ok(pose) => pose,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let costmap = match decode_astar_costmap_with_boundary(costmap_body, boundary.as_ref()) {
        Ok(costmap) => costmap,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let map_resolution = costmap.resolution();
    Module::on_costmap(module, costmap)
        .map_err(|error| module_error("astar_planner.goal_boundary", error))?;
    if let Err(error) = Module::on_goal(module, goal_pose) {
        return module_input_error("astar_planner.goal", error);
    }
    Ok(OwnerInput::Accepted((envelope, boundary, map_resolution)))
}

#[cfg(feature = "simple-2d")]
pub(crate) fn simple_planner_owner(node: &TaskChannels, body_reference_frame_id: Arc<str>) -> NavStackResult<TaskThread> {
    let node = node.clone();
    spawn_task("simple-planner", Box::new(move |shutdown, ready| {
        Box::pin(run_simple_planner(node, body_reference_frame_id, shutdown, ready))
    }))
}

#[cfg(feature = "simple-2d")]
pub(super) async fn run_simple_planner(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_simple_planner::module::{Module, SimplePlannerModule};

    let transport = lcm_transport()?;
    let mut occupancy = typed_stream(&transport, &node.inputs[0], "nav_msgs.OccupancyGrid").await?;
    let mut map_pose =
        typed_stream(&transport, &node.inputs[1], "geometry_msgs.PoseStamped").await?;
    let mut goal_pose =
        typed_stream(&transport, &node.inputs[2], "nuwax_msgs.NavigationGoal").await?;
    let mut stop = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let robot_geometry = Some(
        nuwax_config::derived()
            .map_err(NavStackError::InvalidConfig)?
            .robot_geometry_json
            .clone(),
    )
    .filter(|value| !value.trim().is_empty())
    .map(|value| nuwax_nav_types_messages::RobotGeometry::from_runtime_json(&value))
    .transpose()
    .map_err(NavStackError::InvalidConfig)?;
    let motion_limits = nuwax_nav_types_messages::NavigationCapabilities::from_runtime()
        .map_err(NavStackError::InvalidConfig)?
        .map(|profile| {
            let mut limits = profile.motion;
            let navigation = nuwax_nav_runtime::surface_follower::forward_navigation_kinematics(
                limits.kinematics(),
            );
            limits.model = navigation.model;
            limits.max_reverse_mps = navigation.max_reverse_mps;
            limits.max_lateral_mps = navigation.max_lateral_mps;
            limits
        });
    let mut module =
        SimplePlannerModule::with_config(nuwax_nav_simple_planner::SimplePlannerConfig {
            robot_geometry,
            motion_limits,
            ..nuwax_nav_simple_planner::SimplePlannerConfig::default()
        })
        .map_err(|error| module_error("simple_planner.robot_geometry", error))?;
    module.enable_background_planning();
    let planner_runtime_generation = saturating_i64(runtime_generation());
    let mut latest_goal: Option<schema::NavigationGoal> = None;
    let mut latest_occupancy_body: Option<Vec<u8>> = None;
    let mut active_boundary: Option<NavigationBoundary> = None;
    let mut active_map_resolution = 0.05;
    let mut published_path_is_boundary_safe = false;
    let mut path_revision = 0;
    let mut diagnostics = InputDiagnosticLimiter::default();
    Module::start(&mut module).map_err(|e| module_error("simple_planner", e))?;
    let mut ticker = nuwax_nav_types_messages::clock::interval(
        Duration::from_millis(nuwax_nav_simple_planner::module::TICK_PERIOD_MS), false,
    );
    signal_ready(ready)?;
    loop {
        tokio::select! {
            _=&mut shutdown=>break,
            message = occupancy.next() => {
                let frame = required_frame(message, "simple_planner.occupancy")?;
                let grid = match decode_planner_grid_input(&frame.payload, "simple_planner.occupancy")? {
                    OwnerInput::Accepted(v) => v,
                    OwnerInput::Rejected(e) => {
                        diagnostics.reject("simple_planner.occupancy", &e);
                        continue;
                    }
                };
                let first = latest_occupancy_body.is_none();
                latest_occupancy_body = Some(frame.payload.to_vec());
                active_map_resolution = f64::from(grid.info.resolution);
                published_path_is_boundary_safe = false;
                if first && latest_goal.is_some() {
                    match apply_simple_planner_goal(&mut module, latest_goal.clone().unwrap(), &frame.payload)?
                    {
                        OwnerInput::Accepted((_, boundary)) => {
                            active_boundary = boundary;
                            path_revision = 0;
                        }
                        OwnerInput::Rejected(e) => diagnostics.reject("simple_planner.goal", &e),
                    }
                } else {
                    let occupancy =
                        decode_simple_occupancy_with_boundary(&frame.payload, active_boundary.as_ref())?;
                    module
                        .on_occupancy_grid(occupancy)
                        .map_err(|e| module_error("simple_planner.occupancy", e))?;
                }
            },
            message = map_pose.next() => {
                let frame = latest_pose_frame(message, &mut map_pose, "simple_planner.map_pose")?;
                match decode_planner_pose_input(&frame.payload, "simple_planner.map_pose")? {
                    OwnerInput::Accepted(pose) => Module::on_odometry(&mut module, pose)
                        .map_err(|e| module_error("simple_planner.map_pose", e))?,
                    OwnerInput::Rejected(e) => diagnostics.reject("simple_planner.map_pose", &e),
                }
            },
            message = goal_pose.next() => {
                let frame = required_frame(message, "simple_planner.goal")?;
                let goal = match decode_navigation_goal_input(&frame.payload) {
                    Ok(v) => v,
                    Err(e) => {
                        diagnostics.reject("simple_planner.goal", &e);
                        continue;
                    }
                };
                if latest_goal.as_ref() == Some(&goal) {
                    continue;
                }
                Module::on_stop_movement(&mut module, true)
                    .map_err(|e| module_error("simple_planner.replace", e))?;
                latest_goal = Some(goal.clone());
                active_boundary = None;
                path_revision = 0;
                published_path_is_boundary_safe = false;
                if let Some(body) = &latest_occupancy_body {
                    match apply_simple_planner_goal(&mut module, goal, body)? {
                        OwnerInput::Accepted((_, boundary)) => active_boundary = boundary,
                        OwnerInput::Rejected(e) => diagnostics.reject("simple_planner.goal", &e),
                    }
                }
            },
            message = stop.next() => {
                let frame = required_frame(message, "simple_planner.stop")?;
                match decode_planner_stop_input(&frame.payload, "simple_planner.stop")? {
                    OwnerInput::Accepted(value) => {
                        Module::on_stop_movement(&mut module, value)
                            .map_err(|e| module_error("simple_planner.stop", e))?;
                        if value {
                            latest_goal = None;
                            active_boundary = None;
                            path_revision = 0;
                            published_path_is_boundary_safe = false;
                        }
                    }
                    OwnerInput::Rejected(e) => diagnostics.reject("simple_planner.stop", &e),
                }
            },
            _ = ticker.tick(), if latest_goal.is_some()&&latest_occupancy_body.is_some() => {
                let output =
                    Module::tick(&mut module).map_err(|e| module_tick_error("simple_planner.tick", e))?;
                let mut publication = SimpleOutputPublication {
                    path_yaws: module.planned_yaws(),
                    active_goal: latest_goal.as_ref(),
                    active_boundary: active_boundary.as_ref(),
                    map_resolution: active_map_resolution,
                    planner_runtime_generation,
                    path_revision: &mut path_revision,
                    published_path_is_boundary_safe: &mut published_path_is_boundary_safe,
                };
                publish_simple_output(&transport, &node, output, &mut publication).await?;
            }
        }
    }
    Module::stop(&mut module).map_err(|e| module_error("simple_planner.stop", e))?;
    Ok(())
}

#[cfg(feature = "simple-2d")]
pub(super) struct SimpleOutputPublication<'a> {
    pub(super) path_yaws: Option<&'a [f64]>,
    pub(super) active_goal: Option<&'a schema::NavigationGoal>,
    pub(super) active_boundary: Option<&'a NavigationBoundary>,
    pub(super) map_resolution: f64,
    pub(super) planner_runtime_generation: i64,
    pub(super) path_revision: &'a mut i64,
    pub(super) published_path_is_boundary_safe: &'a mut bool,
}

#[cfg(feature = "simple-2d")]
pub(super) fn simple_hold_path_is_executable(reason: nuwax_nav_simple_planner::module::HoldReason) -> bool {
    reason != nuwax_nav_simple_planner::module::HoldReason::PlanningFailed
}

#[cfg(feature = "simple-2d")]
pub(super) async fn publish_simple_output(
    transport: &LcmTransport,
    node: &TaskChannels,
    output: nuwax_nav_simple_planner::module::ModuleOutput,
    publication: &mut SimpleOutputPublication<'_>,
) -> NavStackResult<()> {
    use nuwax_nav_simple_planner::module::ModuleOutput;

    match output {
        ModuleOutput::Plan { path, waypoint } => {
            let next_revision = publication.path_revision.checked_add(1).ok_or_else(|| {
                NavStackError::Internal("simple planner path revision exhausted".into())
            })?;
            *publication.published_path_is_boundary_safe = publish_versioned_path(
                transport,
                &node.outputs[1],
                VersionedPathPublication {
                    path_yaws: publication.path_yaws,                    goal: publication.active_goal.ok_or_else(|| {
                        NavStackError::Internal(
                            "simple planner path has no active NavigationGoal".into(),
                        )
                    })?,
                    planner_runtime_generation: publication.planner_runtime_generation,
                    path_revision: next_revision,
                    path: &path,
                    boundary_safe: path_respects_navigation_boundary(
                        &path,
                        publication.active_boundary,
                        publication.map_resolution * 0.5,
                    ),
                },
            )
            .await?;
            if *publication.published_path_is_boundary_safe {
                *publication.path_revision = next_revision;
                publish_point(transport, &node.outputs[0], &waypoint).await?;
            }
            Ok(())
        }
        ModuleOutput::Waypoint(waypoint) => {
            let point_is_safe = publication
                .active_boundary
                .is_none_or(|boundary| boundary.contains(waypoint.point.to_2d()));
            if *publication.published_path_is_boundary_safe && point_is_safe {
                publish_point(transport, &node.outputs[0], &waypoint).await?;
            }
            Ok(())
        }
        ModuleOutput::SafetyHold { reason, .. } if !simple_hold_path_is_executable(reason) => {
            *publication.published_path_is_boundary_safe = false;
            Ok(())
        }
        ModuleOutput::SafetyHold { waypoint, path, .. } => {
            let next_revision = publication.path_revision.checked_add(1).ok_or_else(|| {
                NavStackError::Internal("simple planner path revision exhausted".into())
            })?;
            *publication.published_path_is_boundary_safe = publish_versioned_path(
                transport,
                &node.outputs[1],
                VersionedPathPublication {
                    path_yaws: None,                    goal: publication.active_goal.ok_or_else(|| {
                        NavStackError::Internal(
                            "simple planner hold has no active NavigationGoal".into(),
                        )
                    })?,
                    planner_runtime_generation: publication.planner_runtime_generation,
                    path_revision: next_revision,
                    path: &path,
                    boundary_safe: path_respects_navigation_boundary(
                        &path,
                        publication.active_boundary,
                        publication.map_resolution * 0.5,
                    ),
                },
            )
            .await?;
            if *publication.published_path_is_boundary_safe {
                *publication.path_revision = next_revision;
                publish_point(transport, &node.outputs[0], &waypoint).await?;
            }
            Ok(())
        }
        ModuleOutput::Idle => Ok(()),
    }
}



#[cfg(feature = "replanning-astar")]
pub(super) async fn run_astar_planner(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    run_astar_planner_with_transport(node, body_reference_frame_id, transport, shutdown, ready).await
}

#[cfg(feature = "replanning-astar")]
pub(super) async fn run_astar_planner_with_transport(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    transport: LcmTransport,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_astar::module::{
        Module, ModuleOutput, NavigationInterface, ReplanningAStarPlannerModule,
    };

    let mut map_pose =
        typed_stream(&transport, &node.inputs[0], "geometry_msgs.PoseStamped").await?;
    let mut costmap = typed_stream(&transport, &node.inputs[1], "nav_msgs.OccupancyGrid").await?;
    let mut goal = typed_stream(&transport, &node.inputs[2], "nuwax_msgs.NavigationGoal").await?;
    let mut stop = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let value = nuwax_config::derived()
        .map_err(NavStackError::InvalidConfig)?
        .robot_geometry_json
        .clone();
    let robot_geometry = nuwax_nav_types_messages::RobotGeometry::from_runtime_json(&value)
        .map_err(NavStackError::InvalidConfig)?;
    let config = nuwax_nav_astar::GlobalPlannerConfig::new(robot_geometry);
    let mut module = ReplanningAStarPlannerModule::new(config);
    NavigationInterface::start(&mut module)
        .map_err(|error| module_error("astar_planner", error))?;
    let mut has_odometry = false;
    let planner_runtime_generation = saturating_i64(runtime_generation());
    let mut latest_goal: Option<schema::NavigationGoal> = None;
    let mut latest_costmap_body: Option<Vec<u8>> = None;
    let mut active_boundary: Option<NavigationBoundary> = None;
    let mut active_map_resolution = 0.05;
    let mut path_revision = 0_i64;
    let mut diagnostics = InputDiagnosticLimiter::default();
    let mut ticker = nuwax_nav_types_messages::clock::interval(Duration::from_millis(100), false);
    signal_ready(ready)?;
    loop {
        tokio::select! {
            _=&mut shutdown=>break,
            message = map_pose.next() => {
                let frame = latest_pose_frame(message, &mut map_pose, "astar_planner.map_pose")?;
                let pose = match decode_planner_pose_input(&frame.payload, "astar_planner.map_pose")? {
                    OwnerInput::Accepted(v) => v,
                    OwnerInput::Rejected(e) => {
                        diagnostics.reject("astar_planner.map_pose", &e);
                        continue;
                    }
                };
                Module::on_odom(&mut module, pose).map_err(|e| module_error("astar_planner.map_pose", e))?;
                let first = !has_odometry;
                has_odometry = true;
                if first {
                    if let (Some(goal), Some(body)) = (latest_goal.clone(), latest_costmap_body.as_ref()) {
                        match apply_astar_planner_goal(&mut module, goal, body)? {
                            OwnerInput::Accepted((_, boundary, resolution)) => {
                                active_boundary = boundary;
                                active_map_resolution = resolution;
                                path_revision = 0;
                            }
                            OwnerInput::Rejected(e) => diagnostics.reject("astar_planner.goal", &e),
                        }
                    }
                }
            },
            message = costmap.next() => {
                let frame = required_frame(message, "astar_planner.costmap")?;
                match decode_planner_grid_input(&frame.payload, "astar_planner.costmap")? {
                    OwnerInput::Accepted(_) => {}
                    OwnerInput::Rejected(e) => {
                        diagnostics.reject("astar_planner.costmap", &e);
                        continue;
                    }
                }
                let first = latest_costmap_body.is_none();
                latest_costmap_body = Some(frame.payload.to_vec());
                if first && has_odometry && latest_goal.is_some() {
                    let goal = latest_goal.clone().unwrap();
                    let body = &frame.payload;
                    match apply_astar_planner_goal(&mut module, goal, body)? {
                        OwnerInput::Accepted((_, boundary, resolution)) => {
                            active_boundary = boundary;
                            active_map_resolution = resolution;
                            path_revision = 0;
                        }
                        OwnerInput::Rejected(e) => diagnostics.reject("astar_planner.goal", &e),
                    }
                } else {
                    let map = decode_astar_costmap_with_boundary(&frame.payload, active_boundary.as_ref())?;
                    active_map_resolution = map.resolution();
                    Module::on_costmap(&mut module, map)
                        .map_err(|e| module_error("astar_planner.costmap", e))?;
                }
            },
            message = goal.next() => {
                let frame = required_frame(message, "astar_planner.goal")?;
                let goal = match decode_navigation_goal_input(&frame.payload) {
                    Ok(v) => v,
                    Err(e) => {
                        diagnostics.reject("astar_planner.goal", &e);
                        continue;
                    }
                };
                if latest_goal.as_ref() == Some(&goal) {
                    continue;
                }
                Module::on_stop_movement(&mut module, true)
                    .map_err(|e| module_error("astar_planner.replace", e))?;
                latest_goal = Some(goal.clone());
                active_boundary = None;
                path_revision = 0;
                if has_odometry {
                    if let Some(body) = latest_costmap_body.as_ref() {
                        match apply_astar_planner_goal(&mut module, goal, body)? {
                            OwnerInput::Accepted((_, boundary, resolution)) => {
                                active_boundary = boundary;
                                active_map_resolution = resolution;
                                path_revision = 0;
                            }
                            OwnerInput::Rejected(e) => diagnostics.reject("astar_planner.goal", &e),
                        }
                    }
                }
            },
            message = stop.next() => {
                let frame = required_frame(message, "astar_planner.stop")?;
                match decode_planner_stop_input(&frame.payload, "astar_planner.stop")? {
                    OwnerInput::Accepted(value) => {
                        Module::on_stop_movement(&mut module, value)
                            .map_err(|e| module_error("astar_planner.stop", e))?;
                        if value {
                            latest_goal = None;
                            active_boundary = None;
                            path_revision = 0;
                        }
                    }
                    OwnerInput::Rejected(e) => diagnostics.reject("astar_planner.stop", &e),
                }
            },
            _ = ticker.tick(), if has_odometry&&latest_costmap_body.is_some()&&latest_goal.is_some() => {
                let output =
                    Module::tick(&mut module).map_err(|e| module_tick_error("astar_planner.tick", e))?;
                let update = match output {
                    ModuleOutput::Navigation(update) | ModuleOutput::SafetyHold(update) => update,
                };
                let terminal_failure = matches!(
                    update.event,
                    Some(
                        nuwax_nav_astar::global_planner::GlobalPlannerEvent::PlanningFailed
                            | nuwax_nav_astar::global_planner::GlobalPlannerEvent::Blocked
                    )
                );
                let mut boundary_failure = false;
                if let Some(path) = update.path {
                    let next_revision = path_revision
                        .checked_add(1)
                        .ok_or_else(|| NavStackError::Internal("A* planner path revision exhausted".into()))?;
                    let published = publish_versioned_path(
                        &transport,
                        &node.outputs[1],
                        VersionedPathPublication {
                            path_yaws: None,
                            goal: latest_goal.as_ref().ok_or_else(|| {
                                NavStackError::Internal("A* path has no active NavigationGoal".into())
                            })?,
                            planner_runtime_generation,
                            path_revision: next_revision,
                            path: &path,
                            boundary_safe: path_respects_navigation_boundary(
                                &path,
                                active_boundary.as_ref(),
                                active_map_resolution * 0.5,
                            ),
                        },
                    )
                    .await?;
                    if published {
                        path_revision = next_revision;
                    } else {
                        boundary_failure = true;
                    }
                }
                publish_bool(&transport, &node.outputs[2], update.goal_reached).await?;
                if terminal_failure || boundary_failure {
                    publish_bool(&transport, &node.outputs[3], true).await?;
                }
            }
        }
    }
    NavigationInterface::stop(&mut module).map_err(|e| module_error("astar_planner.stop", e))?;
    Ok(())
}



#[cfg(feature = "basic-path-follower")]
pub(super) async fn run_basic_path_follower(
    node: TaskChannels,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_basic_path_follower::module::{
        BasicPathFollowerModule, BasicPathFollowerModuleConfig, Module,
    };

    let transport = lcm_transport()?;
    let mut path = typed_stream(&transport, &node.inputs[0], "nav_msgs.Path").await?;
    let mut odometry = typed_stream(&transport, &node.inputs[1], "nav_msgs.Odometry").await?;
    let mut stop = typed_stream(&transport, &node.inputs[2], "std_msgs.Bool").await?;
    let config = BasicPathFollowerModuleConfig {
        cmd_vel_topic: node.outputs[0].clone(),
        path_topic: node.inputs[0].clone(),
        odom_topic: node.inputs[1].clone(),
        stop_topic: node.inputs[2].clone(),
        ..BasicPathFollowerModuleConfig::default()
    };
    let mut module = BasicPathFollowerModule::new(config)
        .map_err(|error| module_error("basic_path_follower", error))?;
    let stop_input = module.stop_input();
    let mut commands = module.subscribe_cmd_vel();
    let mut goal_reached = module.subscribe_goal_reached();
    Module::start(&mut module).map_err(|error| module_error("basic_path_follower", error))?;
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            message = path.next() => {
                let frame = required_frame(message, "basic_path_follower.path")?;
                Module::on_path(&mut module, decode_path_body(&frame.payload)?);
            }
            message = odometry.next() => {
                let frame = required_frame(message, "basic_path_follower.odometry")?;
                Module::on_odom(&mut module, decode_odometry_pose_body(&frame.payload)?);
            }
            message = stop.next() => {
                let frame = required_frame(message, "basic_path_follower.stop_movement")?;
                let stop = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("stop_movement decode: {error}")))?;
                stop_input.send(stop.data)
                    .map_err(|_| NavStackError::Internal("basic_path_follower stop port closed".into()))?;
            }
            command = commands.recv() => {
                let command = command.ok_or_else(|| NavStackError::Internal(
                    "basic_path_follower command port closed".into()
                ))?;
                publish_twist2(&transport, &node.outputs[0], command).await?;
            }
            reached = goal_reached.recv() => {
                let reached = reached.map_err(|error| NavStackError::Internal(format!(
                    "basic_path_follower goal port: {error}"
                )))?;
                if node.outputs.len() == 2 {
                    publish_bool(&transport, &node.outputs[1], reached).await?;
                }
            }
        }
    }
    Module::stop(&mut module).map_err(|error| module_error("basic_path_follower.stop", error))?;
    publish_twist2(&transport, &node.outputs[0], Twist2::ZERO).await
}

#[cfg(all(test, any(feature = "simple-2d", feature = "replanning-astar")))]
pub(super) fn planner_test_goal(session_id: &str, goal_generation: i64) -> schema::NavigationGoal {
    let mut goal = schema::NavigationGoal {
session_id: session_id.into(),
boundary_points: vec![
            [0.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [5.0, 5.0, 0.0],
            [0.0, 5.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
motion_generation: 1,
goal: schema::PoseStamped {
            header: schema::Header {
                seq: i32::try_from(goal_generation).expect("test generation fits i32"),
                stamp_sec: i32::try_from(goal_generation).expect("test generation fits i32"),
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [4.5, 4.5, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        },
..schema::NavigationGoal::default()
};

    schema::validate_navigation_goal(&goal).expect("seal planner test goal");
    goal
}

#[cfg(all(test, any(feature = "simple-2d", feature = "replanning-astar")))]
pub(super) fn planner_test_grid_body(revision: i32) -> Vec<u8> {
    schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
        header: schema::Header {
            seq: revision,
            stamp_sec: revision,
            frame_id: "map".into(),
            ..schema::Header::default()
        },
        info: schema::MapMetaData {
            resolution: 1.0,
            width: 6,
            height: 6,
            origin: schema::Pose {
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                ..schema::Pose::default()
            },
            ..schema::MapMetaData::default()
        },
        data: vec![0; 36],
    })
    .expect("encode planner test grid")
}
