//! 位姿适配：TF/里程计到地图位姿的转换与发布。

use super::*;

#[cfg(feature = "guard-runtime")]
pub(crate) fn map_pose_owner(
    node: &TaskChannels,
    body_reference_frame_id: Arc<str>,
    lio_motion: Option<(bool, crate::map_pose::LioMotionSender)>,
) -> NavStackResult<TaskThread> {
    let node = node.clone();
    spawn_task("map-pose-adapter", Box::new(move |shutdown, ready| {
        Box::pin(run_map_pose_adapter(node, body_reference_frame_id, lio_motion, shutdown, ready))
    }))
}

#[cfg(feature = "guard-runtime")]
pub(super) fn map_pose_input_error<T>(
    component: &str,
    error: crate::map_pose::MapPoseError,
) -> OwnerInputResult<T> {
    if matches!(&error, crate::map_pose::MapPoseError::InvalidConfig(_)) {
        Err(module_error(component, error))
    } else {
        Ok(OwnerInput::Rejected(error.to_string()))
    }
}

#[cfg(feature = "guard-runtime")]
pub(super) fn ingest_map_pose_tf(
    adapter: &mut crate::map_pose::MapPoseAdapter,
    payload: &[u8],
) -> OwnerInputResult<Option<crate::map_pose::MapPoseObservation>> {
    let message = match schema::decode_tf_message_body(payload) {
        Ok(message) => message,
        Err(error) => return Ok(OwnerInput::Rejected(format!("TFMessage decode: {error}"))),
    };
    match adapter.observe_tf_message(&message) {
        Ok(observed) if observed.accepted => Ok(OwnerInput::Accepted(observed.map_pose)),
        Ok(_) => Ok(OwnerInput::Rejected(
            "TFMessage has no newer canonical map->odom transform".into(),
        )),
        Err(error) => map_pose_input_error("map_pose_adapter.tf", error),
    }
}

#[cfg(feature = "guard-runtime")]
pub(super) enum MapPoseOdometryIngress {
    Accepted(crate::map_pose::MapPoseObservation),
    AwaitingTransform(Box<schema::Odometry>, String),
    Rejected(String),
}

#[cfg(feature = "guard-runtime")]
pub(super) fn transform_map_pose_odometry(
    adapter: &crate::map_pose::MapPoseAdapter,
    odometry: schema::Odometry,
) -> NavStackResult<MapPoseOdometryIngress> {
    match adapter.transform_odometry(&odometry) {
        Ok(Some(observed)) => Ok(MapPoseOdometryIngress::Accepted(observed)),
        Ok(None) => Ok(MapPoseOdometryIngress::AwaitingTransform(
            Box::new(odometry),
            "odometry is waiting for a canonical map->odom transform".into(),
        )),
        Err(error @ crate::map_pose::MapPoseError::MisalignedTransform { .. }) => {
            Ok(MapPoseOdometryIngress::AwaitingTransform(
                Box::new(odometry),
                error.to_string(),
            ))
        }
        Err(error @ crate::map_pose::MapPoseError::InvalidConfig(_)) => {
            Err(module_error("map_pose_adapter.odometry", error))
        }
        Err(error) => Ok(MapPoseOdometryIngress::Rejected(error.to_string())),
    }
}

#[cfg(feature = "guard-runtime")]
pub(super) fn ingest_map_pose_odometry(
    adapter: &crate::map_pose::MapPoseAdapter,
    payload: &[u8],
) -> NavStackResult<MapPoseOdometryIngress> {
    let odometry = match schema::decode_odometry(&prefixed(schema::ODOMETRY_FINGERPRINT, payload)) {
        Ok(odometry) => odometry,
        Err(error) => {
            return Ok(MapPoseOdometryIngress::Rejected(format!(
                "Odometry decode: {error}"
            )))
        }
    };
    transform_map_pose_odometry(adapter, odometry)
}

#[cfg(feature = "guard-runtime")]
pub(super) async fn publish_map_pose_observation(
    transport: &LcmTransport,
    channel: &str,
    observed: crate::map_pose::MapPoseObservation,
    motion: Option<&mut crate::map_pose::LioMotionEstimator>,
) -> NavStackResult<()> {
    let body = schema::encode_pose_stamped_body(&observed.pose)
        .map_err(|e| NavStackError::Internal(e.to_string()))?;
    if let Some(motion) = motion {
        motion.observe(observed.odometry);
    }
    publish_body(transport, channel, "geometry_msgs.PoseStamped", body).await
}

#[cfg(feature = "guard-runtime")]
pub(super) fn bound_map_pose_adapter(
    map_frame: String,
    odom_frame: String,
    body_reference_frame_id: &str,
) -> NavStackResult<crate::map_pose::MapPoseAdapter> {
    crate::map_pose::MapPoseAdapter::new(map_frame, odom_frame, body_reference_frame_id)
        .map(|adapter| adapter.require_body_reference())
        .map_err(|error| module_error("map_pose_adapter", error))
}

#[cfg(feature = "guard-runtime")]
pub(super) async fn run_map_pose_adapter(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    lio_motion: Option<(bool, crate::map_pose::LioMotionSender)>,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    let mut odometry = typed_stream(&transport, &node.inputs[0], "nav_msgs.Odometry").await?;
    let mut transforms = typed_stream(&transport, &node.inputs[1], "tf2_msgs.TFMessage").await?;
    let mut adapter = bound_map_pose_adapter(
        nuwax_config::robot()
            .map_err(NavStackError::InvalidConfig)?
            .map_frame
            .clone(),
        nuwax_config::robot()
            .map_err(NavStackError::InvalidConfig)?
            .odom_frame
            .clone(),
        &body_reference_frame_id,
    )?;
    let mut diagnostics = InputDiagnosticLimiter::default();
    let mut pending_odometry: Option<Box<schema::Odometry>> = None;
    let mut motion = match lio_motion {
        Some((native_twist, output)) => Some(crate::map_pose::LioMotionEstimator::new(
            native_twist, configured_pose_source_max_age()?, body_reference_frame_id, output,
        )),
        None => None,
    };
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => return Ok(()),
            message = transforms.next() => {
                let mut next = message;
                let mut newest = None;
                loop {
                    let frame = required_frame(next, "map_pose_adapter.tf")?;
                    match ingest_map_pose_tf(&mut adapter, &frame.payload)? {
                        OwnerInput::Accepted(Some(observed)) => {
                            pending_odometry=None;
                            newest=Some(observed);
                        }
                        OwnerInput::Accepted(None) => {
                            if let Some(odometry) = pending_odometry.take() {
                                match transform_map_pose_odometry(&adapter, *odometry)? {
                                    MapPoseOdometryIngress::Accepted(observed) => {
                                        newest=Some(observed);
                                    }
                                    MapPoseOdometryIngress::AwaitingTransform(odometry, reason) => {
                                        pending_odometry = Some(odometry);
                                        diagnostics.reject("map_pose_adapter.odometry", &reason);
                                    }
                                    MapPoseOdometryIngress::Rejected(reason) => {
                                        diagnostics.reject("map_pose_adapter.odometry", &reason);
                                    }
                                }
                            }
                        }
                        OwnerInput::Rejected(reason) => diagnostics.reject("map_pose_adapter.tf", &reason),
                    }
                    match transforms.next().now_or_never() {
                        Some(message) => next = message,
                        None => break,
                    }
                }
                if let Some(observed) = newest {
                    publish_map_pose_observation(&transport, &node.outputs[0],
                        observed, motion.as_mut()).await?;
                }
            }
            message = odometry.next() => {
                let mut newest = None;
                {
                    let frame = latest_pose_frame(message, &mut odometry, "map_pose_adapter.odometry")?;
                    match ingest_map_pose_odometry(&adapter, &frame.payload)? {
                        MapPoseOdometryIngress::Accepted(observed) => {
                            pending_odometry=None;
                            newest=Some(observed);
                        }
                        MapPoseOdometryIngress::AwaitingTransform(candidate, reason) => {
                            pending_odometry=Some(candidate);
                            diagnostics.reject("map_pose_adapter.odometry", &reason);
                        }
                        MapPoseOdometryIngress::Rejected(reason) => {
                            diagnostics.reject("map_pose_adapter.odometry", &reason);
                        }
                    }
                }
                if let Some(observed) = newest {
                    publish_map_pose_observation(&transport, &node.outputs[0],
                        observed, motion.as_mut()).await?;
                }
            }
        }
    }
}
