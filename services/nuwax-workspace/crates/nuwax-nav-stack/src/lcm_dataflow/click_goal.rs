//! 点击目标：地图点击转导航目标。

use super::*;

#[cfg(feature = "click-goal")]
pub(super) async fn run_click_goal(
    node: TaskChannels,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    let mut clicks =
        typed_stream(&transport, &node.inputs[0], "geometry_msgs.PointStamped").await?;
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            message = clicks.next() => {
                let frame = required_frame(message, "click_goal_router.clicked_point")?;
                let click = schema::decode_point_stamped_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("clicked_point decode: {error}")))?;
                let goal = clicked_goal_pose(&click)?;
                publish_pose(&transport, &node.outputs[0], &goal).await?;
            }
        }
    }
    Ok(())
}

#[cfg(feature = "click-goal")]
pub(super) fn clicked_goal_pose(click: &schema::PointStamped) -> NavStackResult<PoseStamped> {
    if click.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "clicked goal frame must be map, got {}",
            click.header.frame_id,
        )));
    }
    lcm_header_timestamp_ns(&click.header, "click_goal_router.clicked_point")?;
    let mut goal = PoseStamped::from_pose(
        Pose3::from_pose2(nuwax_nav_types_messages::Pose2::new(
            click.point[0],
            click.point[1],
            0.0,
        )),
        NavFrameId::Map,
    );
    goal.stamp = Timestamp::from_unix(
        i64::from(click.header.stamp_sec),
        click.header.stamp_nsec as u32,
    );
    Ok(goal)
}
