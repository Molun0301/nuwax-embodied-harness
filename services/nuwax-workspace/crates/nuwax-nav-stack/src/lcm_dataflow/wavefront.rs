//! 二维波前探索。

use super::*;

#[cfg(feature = "explore")]
pub(super) async fn run_wavefront(
    node: TaskChannels,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_explore_wavefront::runtime::{
        RuntimeInputs, RuntimeTickOutcome, WavefrontExplorerRuntime,
    };
    use nuwax_explore_wavefront::WavefrontGoalSelector;

    let transport = lcm_transport()?;
    let mut maps = typed_stream(&transport, &node.inputs[0], "nav_msgs.OccupancyGrid").await?;
    let mut odometry = typed_stream(&transport, &node.inputs[1], "nav_msgs.Odometry").await?;
    let mut reached = typed_stream(&transport, &node.inputs[2], "std_msgs.Bool").await?;
    let mut failed = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let mut stop = typed_stream(&transport, &node.inputs[4], "std_msgs.Bool").await?;

    let config = wavefront_config_from_runtime()?;
    let mut runtime = WavefrontExplorerRuntime::new(
        WavefrontGoalSelector::new(config.clone()),
        LcmWavefrontGoalSink,
        WavefrontGoalIds::default(),
        config,
    );
    runtime
        .begin()
        .map_err(|error| module_error("wavefront.begin", format!("{error:?}")))?;
    let mut latest_map = None;
    let mut latest_odometry = None;
    let mut ticker = nuwax_nav_types_messages::clock::interval(nuwax_explore_wavefront::runtime::NO_INPUT_TICK, false);
    signal_ready(ready)?;

    loop {
        tokio::select! {
            biased;
            _ = &mut shutdown => break,
            message = maps.next() => {
                let frame = required_frame(message, "wavefront.occupancy_grid")?;
                latest_map = Some(decode_wavefront_grid(&frame.payload)?);
            }
            message = odometry.next() => {
                let frame = required_frame(message, "wavefront.odometry")?;
                let pose = decode_odometry_pose_body(&frame.payload)?;
                if pose.frame_id != NavFrameId::Map {
                    return Err(NavStackError::InvalidConfig(format!(
                        "wavefront odometry frame must be map, got {}",
                        pose.frame_id.as_str()
                    )));
                }
                latest_odometry = Some(pose);
            }
            message = reached.next() => {
                let frame = required_frame(message, "wavefront.goal_reached")?;
                let feedback = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("wavefront goal_reached decode: {error}")))?;
                if feedback.data {
                    runtime.report_goal_reached(true);
                }
            }
            message = failed.next() => {
                let frame = required_frame(message, "wavefront.goal_failed")?;
                let feedback = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("wavefront goal_failed decode: {error}")))?;
                if feedback.data {
                    runtime.report_goal_reached(false);
                }
            }
            message = stop.next() => {
                let frame = required_frame(message, "wavefront.stop_movement")?;
                let requested = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("wavefront stop_movement decode: {error}")))?;
                if requested.data {
                    runtime.request_stop_movement()
                        .map_err(|error| module_error("wavefront.stop_movement", format!("{error:?}")))?;
                }
            }
            _ = ticker.tick() => {
                let outcome = runtime
                    .tick(RuntimeInputs {
                        costmap: latest_map.as_ref(),
                        odom: latest_odometry.as_ref(),
                        now: unix_timestamp(),
                    })
                    .map_err(|error| module_error("wavefront.tick", format!("{error:?}")))?;
                if let RuntimeTickOutcome::Goal(goal) = outcome {
                    publish_pose(&transport, &node.outputs[0], &goal.pose).await?;
                }
            }
        }
    }

    if runtime.is_active() {
        runtime
            .request_stop_movement()
            .map_err(|error| module_error("wavefront.shutdown", format!("{error:?}")))?;
        let _ = runtime
            .tick(RuntimeInputs {
                costmap: latest_map.as_ref(),
                odom: latest_odometry.as_ref(),
                now: unix_timestamp(),
            })
            .map_err(|error| module_error("wavefront.shutdown", format!("{error:?}")))?;
    }
    if runtime.is_active() {
        runtime
            .end()
            .map_err(|error| module_error("wavefront.end", format!("{error:?}")))?;
    }
    Ok(())
}

#[cfg(feature = "explore")]
pub(super) fn wavefront_config_from_runtime() -> NavStackResult<nuwax_explore_wavefront::WavefrontConfig> {
    Ok(nuwax_explore_wavefront::WavefrontConfig {
        goal_timeout_ms: nuwax_config::navigation()
            .map_err(NavStackError::InvalidConfig)?
            .wavefront_goal_timeout_ms as u64,
        ..Default::default()
    })
}

#[cfg(feature = "explore")]
pub(super) struct LcmWavefrontGoalSink;

#[cfg(feature = "explore")]
impl nuwax_explore_wavefront::runtime::WavefrontGoalSink for LcmWavefrontGoalSink {
    fn publish(&mut self, _goal: nuwax_nav_types_messages::FrontierGoal) {}
}

#[cfg(feature = "explore")]
#[derive(Default)]
pub(super) struct WavefrontGoalIds(u64);

#[cfg(feature = "explore")]
impl nuwax_explore_wavefront::FrontierGoalIdSource for WavefrontGoalIds {
    fn next_frontier_goal_id(
        &mut self,
        seed: &nuwax_explore_wavefront::FrontierGoalIdSeed,
    ) -> String {
        self.0 = self.0.saturating_add(1);
        format!("wavefront-{}-{}", seed.stamp.nanos, self.0)
    }
}

#[cfg(feature = "explore")]
pub(super) fn decode_wavefront_grid(body: &[u8]) -> NavStackResult<OccupancyGrid2D> {
    let grid = schema::decode_occupancy_grid_body(body)
        .map_err(|error| NavStackError::Internal(format!("OccupancyGrid decode: {error}")))?;
    timestamp(&grid.header)?;
    if grid.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "wavefront OccupancyGrid frame must be map, got {}",
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
            "wavefront grid cannot represent a rotated or non-planar origin".into(),
        ));
    }
    let width = u32::try_from(grid.info.width)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid width is negative".into()))?;
    let height = u32::try_from(grid.info.height)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid height is negative".into()))?;
    let resolution = f64::from(grid.info.resolution);
    let origin = Point2::new(grid.info.origin.position[0], grid.info.origin.position[1]);
    if width == 0
        || height == 0
        || !resolution.is_finite()
        || resolution <= 0.0
        || !origin.x.is_finite()
        || !origin.y.is_finite()
    {
        return Err(NavStackError::InvalidConfig(
            "wavefront grid dimensions, resolution, and origin must be finite and valid".into(),
        ));
    }
    let expected = (width as usize).saturating_mul(height as usize);
    if grid.data.len() != expected {
        return Err(NavStackError::InvalidConfig(format!(
            "OccupancyGrid has {} cells, expected {expected}",
            grid.data.len()
        )));
    }
    let cells = grid
        .data
        .into_iter()
        .map(|value| match value {
            value if value < 0 => OccupancyCell::Unknown,
            0 => OccupancyCell::Free,
            value if value >= 100 => OccupancyCell::Occupied,
            value => OccupancyCell::Cost(value as u8),
        })
        .collect();
    Ok(OccupancyGrid2D::new(
        origin, resolution, width, height, cells,
    ))
}

#[cfg(feature = "explore")]
pub(super) fn unix_timestamp() -> Timestamp {
    let now = nuwax_nav_types_messages::clock::source_now_ns();
    Timestamp::from_unix((now / 1_000_000_000) as i64, (now % 1_000_000_000) as u32)
}
