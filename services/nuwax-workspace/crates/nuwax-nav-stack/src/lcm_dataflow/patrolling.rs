//! 巡逻。

use super::*;

#[cfg(feature = "patrolling")]
pub(super) const PLANNER_CONTROL_TIMEOUT: Duration = Duration::from_secs(2);

#[cfg(feature = "patrolling")]
pub(super) async fn run_patrolling(
    node: TaskChannels,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_patrolling::{
        create_patrol_router, Module, PatrolRouterName, PatrollingModule, PatrollingModuleConfig,
    };

    let transport = lcm_transport()?;
    let mut map_pose =
        typed_stream(&transport, &node.inputs[0], "geometry_msgs.PoseStamped").await?;
    let mut costmap = typed_stream(&transport, &node.inputs[1], "nav_msgs.OccupancyGrid").await?;
    let mut goal_reached = typed_stream(&transport, &node.inputs[2], "std_msgs.Bool").await?;
    let mut activation = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let mut goal_failed = typed_stream(&transport, &node.inputs[4], "std_msgs.Bool").await?;
    let mut planner_ack = typed_stream(&transport, &node.inputs[5], "std_msgs.Bool").await?;
    let mut module = PatrollingModule::new(
        create_patrol_router(PatrolRouterName::Coverage, 0.3, 32, Some(0)),
        PatrollingModuleConfig::default(),
    );
    let mut goals = module.subscribe_goals();
    let mut holds = module.subscribe_hold();
    signal_ready(ready)?;

    loop {
        tokio::select! {
            biased;
            _ = &mut shutdown => break,
            message = activation.next() => {
                let frame = required_frame(message, "patrolling.activation")?;
                let enabled = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("patrolling activation decode: {error}")))?
                    .data;
                if enabled {
                    if !Module::running(&module) {
                        request_patrol_planner_control(
                            &transport,
                            &node.outputs[3],
                            &mut planner_ack,
                            true,
                        ).await?;
                        Module::start(&mut module);
                        if !Module::running(&module) {
                            let _ = request_patrol_planner_control(
                                &transport,
                                &node.outputs[3],
                                &mut planner_ack,
                                false,
                            ).await;
                            return Err(NavStackError::Internal(
                                "patrolling lifecycle failed to start".into(),
                            ));
                        }
                    }
                } else {
                    let restore = request_patrol_planner_control(
                        &transport,
                        &node.outputs[3],
                        &mut planner_ack,
                        false,
                    ).await;
                    let stopped = stop_patrolling(
                        &transport,
                        &node,
                        &mut module,
                        &mut goals,
                        &mut holds,
                    ).await;
                    restore?;
                    stopped?;
                }
            }
            message = map_pose.next() => {
                let frame = required_frame(message, "patrolling.map_pose")?;
                let map_pose = decode_pose_body(&frame.payload)?;
                require_map_pose(&map_pose, "patrolling.map_pose")?;
                module.update_odometry(map_pose);
            }
            message = costmap.next() => {
                let frame = required_frame(message, "patrolling.global_costmap")?;
                module.update_grid(decode_patrol_grid(&frame.payload)?);
            }
            message = goal_reached.next() => {
                let frame = required_frame(message, "patrolling.goal_reached")?;
                let reached = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("patrolling goal_reached decode: {error}")))?;
                if reached.data {
                    Module::report_goal_reached(&mut module, true);
                }
            }
            message = goal_failed.next() => {
                let frame = required_frame(message, "patrolling.goal_failed")?;
                let failed = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("patrolling goal_failed decode: {error}")))?;
                if failed.data && Module::running(&module) {
                    Module::report_goal_failed(&mut module);
                    if !Module::running(&module) {
                        let restore = request_patrol_planner_control(
                            &transport,
                            &node.outputs[3],
                            &mut planner_ack,
                            false,
                        ).await;
                        let stopped = publish_patrolling_stop_outputs(
                            &transport,
                            &node,
                            &mut goals,
                            &mut holds,
                        ).await;
                        restore?;
                        stopped?;
                    }
                }
            }
            goal = goals.recv() => {
                let goal = goal.map_err(|error| NavStackError::Internal(format!(
                    "patrolling goal port: {error}"
                )))?;
                publish_pose(&transport, &node.outputs[0], &goal).await?;
            }
        }
    }

    let _ = publish_bool(&transport, &node.outputs[3], false).await;
    stop_patrolling(&transport, &node, &mut module, &mut goals, &mut holds).await
}

#[cfg(feature = "patrolling")]
pub(super) async fn stop_patrolling(
    transport: &LcmTransport,
    node: &TaskChannels,
    module: &mut nuwax_nav_patrolling::PatrollingModule,
    goals: &mut tokio::sync::broadcast::Receiver<PoseStamped>,
    holds: &mut tokio::sync::broadcast::Receiver<Twist2>,
) -> NavStackResult<()> {
    use nuwax_nav_patrolling::Module;

    let was_running = Module::running(module);
    Module::stop(module);
    if !was_running {
        return Ok(());
    }

    publish_patrolling_stop_outputs(transport, node, goals, holds).await
}

#[cfg(feature = "patrolling")]
pub(super) async fn publish_patrolling_stop_outputs(
    transport: &LcmTransport,
    node: &TaskChannels,
    goals: &mut tokio::sync::broadcast::Receiver<PoseStamped>,
    holds: &mut tokio::sync::broadcast::Receiver<Twist2>,
) -> NavStackResult<()> {
    let hold = latest_broadcast(holds).unwrap_or(Twist2::ZERO);
    publish_twist2(transport, &node.outputs[1], hold).await?;
    publish_bool(transport, &node.outputs[2], true).await?;
    if let Some(goal) = latest_broadcast(goals) {
        publish_pose(transport, &node.outputs[0], &goal).await?;
    }
    Ok(())
}

#[cfg(feature = "patrolling")]
pub(super) async fn request_patrol_planner_control(
    transport: &LcmTransport,
    request_channel: &str,
    acknowledgements: &mut LcmFrameStream,
    active: bool,
) -> NavStackResult<()> {
    publish_bool(transport, request_channel, active).await?;
    let result = match tokio::time::timeout(PLANNER_CONTROL_TIMEOUT, async {
        loop {
            let frame = required_frame(
                acknowledgements.next().await,
                "patrolling.planner_control_ack",
            )?;
            let acknowledged = schema::decode_bool_body(&frame.payload).map_err(|error| {
                NavStackError::Internal(format!("patrolling planner control ack decode: {error}"))
            })?;
            if acknowledged.data == active {
                return Ok(());
            }
        }
    })
    .await
    {
        Ok(result) => result,
        Err(_) => Err(NavStackError::Internal(format!(
            "patrolling planner control ack timed out for active={active}"
        ))),
    };
    if result.is_err() && active {
        let _ = publish_bool(transport, request_channel, false).await;
    }
    result
}

#[cfg(feature = "patrolling")]
pub(super) fn latest_broadcast<T>(receiver: &mut tokio::sync::broadcast::Receiver<T>) -> Option<T>
where
    T: Clone,
{
    let mut latest = None;
    loop {
        match receiver.try_recv() {
            Ok(value) => latest = Some(value),
            Err(tokio::sync::broadcast::error::TryRecvError::Lagged(_)) => continue,
            Err(
                tokio::sync::broadcast::error::TryRecvError::Empty
                | tokio::sync::broadcast::error::TryRecvError::Closed,
            ) => return latest,
        }
    }
}

#[cfg(feature = "patrolling")]
pub(super) fn decode_patrol_grid(body: &[u8]) -> NavStackResult<GridSnapshot> {
    let grid = schema::decode_occupancy_grid_body(body)
        .map_err(|error| NavStackError::Internal(format!("OccupancyGrid decode: {error}")))?;
    timestamp(&grid.header)?;
    if grid.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "patrolling OccupancyGrid frame must be map, got {}",
            grid.header.frame_id
        )));
    }
    let orientation = grid.info.origin.orientation_xyzw;
    let identity_rotation = orientation[0].abs() <= 1e-9
        && orientation[1].abs() <= 1e-9
        && orientation[2].abs() <= 1e-9
        && (orientation[3].abs() - 1.0).abs() <= 1e-9;
    if !identity_rotation || grid.info.origin.position[2].abs() > 1e-9 {
        return Err(NavStackError::InvalidConfig(
            "patrolling grid cannot represent a rotated or non-planar origin".into(),
        ));
    }
    let width = u32::try_from(grid.info.width)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid width is negative".into()))?;
    let height = u32::try_from(grid.info.height)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid height is negative".into()))?;
    let snapshot = GridSnapshot {
        width,
        height,
        resolution: f64::from(grid.info.resolution),
        origin_x: grid.info.origin.position[0],
        origin_y: grid.info.origin.position[1],
        data: grid.data,
    };
    snapshot
        .validate()
        .map_err(|reason| NavStackError::InvalidConfig(reason.into()))?;
    if !(snapshot.resolution.is_finite()
        && snapshot.origin_x.is_finite()
        && snapshot.origin_y.is_finite())
    {
        return Err(NavStackError::InvalidConfig(
            "patrolling grid metadata must be finite".into(),
        ));
    }
    Ok(snapshot)
}
