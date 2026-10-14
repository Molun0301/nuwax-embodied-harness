//! Real-LCM production owner for the person-follow lifecycle.

use std::future::Future;
use std::sync::Arc;
use std::time::Instant;

use futures::StreamExt;
use nuwax_codec::lcm::{person_follow, person_follow_command, schema};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame, RawLcmFrameBuilder};
use nuwax_tf::lcm::{LcmTf, LcmTfError};
use nuwax_tf::{make_transform, TfTree, TransformStamped};
use nuwax_transport::{LcmFrameStream, LcmTransport, TransportError};
use thiserror::Error;

use crate::person_follow::{
    PackedPointCloud, PersonFollowConfig, PersonFollowController, PersonFollowError,
    PersonFollowOutput, PersonFollowState, TargetStamp, TrackedPerson, TrackingState,
};
use crate::{CameraIntrinsics, DetectionNavigationConfig, PoseStamped, TfLike};

/// Default typed tracked-person input channel.
pub const DEFAULT_TARGET_CHANNEL: &str = "/person_follow_target";
/// Default typed global PointCloud2 input channel.
pub const DEFAULT_POINTCLOUD_CHANNEL: &str = "/global_map";
/// Default typed TF input channel.
pub const DEFAULT_TF_CHANNEL: &str = "/tf";
/// Gateway-owned person-follow command channel.
pub const DEFAULT_COMMAND_CHANNEL: &str = "/gateway/person_follow_cmd_vel";

const TARGET_TYPE: &str = "nuwax_msgs.PersonFollowTarget";
const POINTCLOUD_TYPE: &str = "sensor_msgs.PointCloud2";
const COMMAND_TYPE: &str = "nuwax_msgs.PersonFollowCommand";
const TF_TYPED_SUFFIX: &str = "tf2_msgs.TFMessage";

/// Real-LCM owner configuration.
#[derive(Debug, Clone)]
pub struct PersonFollowLcmConfig {
    /// Tracked-person input channel without its typed suffix.
    pub target_channel: String,
    /// Global PointCloud2 input channel without its typed suffix.
    pub pointcloud_channel: String,
    /// TF channel without its typed suffix.
    pub tf_channel: String,
    /// Output channel consumed by the Gateway guard client.
    pub command_channel: String,
    /// TF history duration.
    pub tf_buffer_size_secs: f64,
    /// Person-follow lifecycle settings.
    pub controller: PersonFollowConfig,
    /// DetectionNavigation controller settings.
    pub navigation: DetectionNavigationConfig,
}

impl Default for PersonFollowLcmConfig {
    fn default() -> Self {
        Self {
            target_channel: DEFAULT_TARGET_CHANNEL.into(),
            pointcloud_channel: DEFAULT_POINTCLOUD_CHANNEL.into(),
            tf_channel: DEFAULT_TF_CHANNEL.into(),
            command_channel: DEFAULT_COMMAND_CHANNEL.into(),
            tf_buffer_size_secs: 10.0,
            controller: PersonFollowConfig::default(),
            navigation: DetectionNavigationConfig::default(),
        }
    }
}

/// Production owner failures. Every runtime failure path attempts a zero command first.
#[derive(Debug, Error)]
pub enum PersonFollowLcmError {
    /// Invalid static owner configuration.
    #[error("invalid person-follow LCM config: {0}")]
    InvalidConfig(String),
    /// Typed transport error.
    #[error(transparent)]
    Transport(#[from] TransportError),
    /// Generated message codec error.
    #[error(transparent)]
    Codec(#[from] nuwax_codec::CodecError),
    /// TF subscription lifecycle error.
    #[error(transparent)]
    Tf(#[from] LcmTfError),
    /// Person-follow input or controller error.
    #[error(transparent)]
    Controller(#[from] PersonFollowError),
    /// A typed subscription ended unexpectedly.
    #[error("{0} subscription closed unexpectedly")]
    SubscriptionClosed(&'static str),
}

/// Result alias for the real-LCM owner.
pub type PersonFollowLcmResult<T> = Result<T, PersonFollowLcmError>;

/// Real-LCM owner with one shared liblcm event context for target, cloud, TF, and command I/O.
pub struct PersonFollowLcmOwner {
    transport: LcmTransport,
    config: PersonFollowLcmConfig,
}

impl PersonFollowLcmOwner {
    /// Validate command ownership and construct an owner around a real transport.
    pub fn new(
        transport: LcmTransport,
        config: PersonFollowLcmConfig,
    ) -> PersonFollowLcmResult<Self> {
        validate_config(&config)?;
        if transport.shared_lcm_context().is_none() {
            return Err(PersonFollowLcmError::InvalidConfig(
                "transport does not expose a real libLCM context".into(),
            ));
        }
        Ok(Self { transport, config })
    }

    /// Run until shutdown or a typed runtime failure.
    ///
    /// The owner publishes zero on explicit stop/loss and again during all teardown paths.
    pub async fn run_until<F>(self, shutdown: F) -> PersonFollowLcmResult<()>
    where
        F: Future<Output = ()> + Send,
    {
        self.run_until_with_ready(shutdown, || Ok(())).await
    }

    /// Run until shutdown and signal readiness after every typed input is subscribed.
    pub async fn run_until_with_ready<F, R>(
        self,
        shutdown: F,
        ready: R,
    ) -> PersonFollowLcmResult<()>
    where
        F: Future<Output = ()> + Send,
        R: FnOnce() -> PersonFollowLcmResult<()> + Send,
    {
        let context = self.transport.shared_lcm_context().ok_or_else(|| {
            PersonFollowLcmError::InvalidConfig("transport lost its real libLCM context".into())
        })?;
        let typed_tf_channel = format!("{}#{TF_TYPED_SUFFIX}", self.config.tf_channel);
        let tf = LcmTf::from_context(context, &typed_tf_channel, self.config.tf_buffer_size_secs)
            .await?;
        let tree = tf.tree();
        let mut controller = PersonFollowController::new(
            Box::new(TfTreeSource::new(Arc::clone(&tree))),
            self.config.navigation.clone(),
            self.config.controller,
        )?;
        let targets =
            typed_stream(&self.transport, &self.config.target_channel, TARGET_TYPE).await?;
        let pointclouds = typed_stream(
            &self.transport,
            &self.config.pointcloud_channel,
            POINTCLOUD_TYPE,
        )
        .await?;

        let runtime_result = match ready() {
            Ok(()) => {
                self.run_loop(&mut controller, tree, targets, pointclouds, shutdown)
                    .await
            }
            Err(error) => Err(error),
        };
        let terminal_result = match controller.teardown() {
            Some(output) => {
                publish_output(&self.transport, &self.config.command_channel, output).await
            }
            None => Ok(()),
        };
        let tf_result = tf.stop().await.map_err(PersonFollowLcmError::from);
        runtime_result?;
        terminal_result?;
        tf_result
    }

    async fn run_loop<F>(
        &self,
        controller: &mut PersonFollowController,
        tree: Arc<TfTree>,
        mut targets: LcmFrameStream,
        mut pointclouds: LcmFrameStream,
        shutdown: F,
    ) -> PersonFollowLcmResult<()>
    where
        F: Future<Output = ()> + Send,
    {
        let started = Instant::now();
        let mut control_tick = tokio::time::interval(controller.control_period());
        control_tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        tokio::pin!(shutdown);

        loop {
            tokio::select! {
                biased;
                _ = &mut shutdown => return Ok(()),
                frame = targets.next() => {
                    let frame = required_frame(frame, "person-follow target")?;
                    let decoded = match decode_target(&frame.payload) {
                        Ok(target) => target,
                        Err(error) => {
                            tracing::warn!(%error, "rejecting invalid person-follow target");
                            continue;
                        }
                    };
                    match controller.accept_target(
                        decoded.target,
                        decoded.target_seq,
                        decoded.required_cloud_revision,
                        decoded.stamp,
                        started.elapsed(),
                    ) {
                        Ok(true) => tree.set(decoded.calibration),
                        Ok(false) => tracing::warn!("rejecting stale, duplicate, or cross-session target"),
                        Err(error) => tracing::warn!(%error, "rejecting invalid person-follow target"),
                    }
                }
                frame = pointclouds.next() => {
                    let frame = required_frame(frame, "person-follow pointcloud")?;
                    match decode_pointcloud_body(&frame.payload)
                        .and_then(|cloud| PackedPointCloud::from_lcm(&cloud).map_err(Into::into))
                    {
                        Ok(cloud) => controller.update_pointcloud(cloud),
                        Err(error) => {
                            tracing::warn!(%error, "rejecting invalid global pointcloud");
                            if let Some(output) = controller.fail_active(format!(
                                "invalid global pointcloud: {error}"
                            )) {
                                publish_output(
                                    &self.transport,
                                    &self.config.command_channel,
                                    output,
                                )
                                .await?;
                            }
                        }
                    }
                }
                _ = control_tick.tick() => {
                    if let Some(output) = controller.tick(started.elapsed()) {
                        publish_output(&self.transport, &self.config.command_channel, output).await?;
                    }
                }
            }
        }
    }
}

fn validate_config(config: &PersonFollowLcmConfig) -> PersonFollowLcmResult<()> {
    for (label, channel) in [
        ("target", &config.target_channel),
        ("pointcloud", &config.pointcloud_channel),
        ("tf", &config.tf_channel),
        ("command", &config.command_channel),
    ] {
        LcmTopic::new(channel).map_err(|error| {
            PersonFollowLcmError::InvalidConfig(format!("{label} channel: {error}"))
        })?;
    }
    if matches!(config.command_channel.as_str(), "/cmd_vel" | "/nav_cmd_vel") {
        return Err(PersonFollowLcmError::InvalidConfig(
            "person follow must publish to the Gateway ingress; direct cmd_vel and legacy nav_cmd_vel are forbidden"
                .into(),
        ));
    }
    if !(config.tf_buffer_size_secs.is_finite() && config.tf_buffer_size_secs > 0.0) {
        return Err(PersonFollowLcmError::InvalidConfig(
            "tf_buffer_size_secs must be finite and positive".into(),
        ));
    }
    if config.controller.control_frequency_hz == 0 || config.controller.target_stale_after.is_zero()
    {
        return Err(PersonFollowLcmError::InvalidConfig(
            "controller frequency and target timeout must be positive".into(),
        ));
    }
    Ok(())
}

async fn typed_stream(
    transport: &LcmTransport,
    channel: &str,
    kind: &str,
) -> PersonFollowLcmResult<LcmFrameStream> {
    let topic = LcmTopic::new(channel)
        .map_err(|error| PersonFollowLcmError::InvalidConfig(error.to_string()))?;
    let message_type = LcmMessageType::new(kind)
        .map_err(|error| PersonFollowLcmError::InvalidConfig(error.to_string()))?;
    Ok(transport.subscribe(topic, Some(message_type)).await)
}

fn required_frame(
    frame: Option<Result<RawLcmFrame, TransportError>>,
    label: &'static str,
) -> PersonFollowLcmResult<RawLcmFrame> {
    frame
        .ok_or(PersonFollowLcmError::SubscriptionClosed(label))?
        .map_err(PersonFollowLcmError::from)
}

#[derive(Debug)]
struct DecodedTarget {
    target: TrackedPerson,
    target_seq: i64,
    required_cloud_revision: i64,
    stamp: TargetStamp,
    calibration: TransformStamped,
}

fn decode_target(body: &[u8]) -> PersonFollowLcmResult<DecodedTarget> {
    let message = person_follow::decode_person_follow_target_body(body)?;
    if message.target_seq <= 0 || message.required_cloud_revision <= 0 {
        return Err(invalid_target(
            "target_seq and required_cloud_revision must be positive",
        ));
    }
    if !(0..1_000_000_000).contains(&message.stamp_nsec) {
        return Err(PersonFollowLcmError::InvalidConfig(format!(
            "target stamp_nsec {} is outside [0, 1e9)",
            message.stamp_nsec
        )));
    }
    if message.stamp_sec < 0 {
        return Err(invalid_target("stamp_sec must be non-negative"));
    }
    for (name, value) in [
        ("session_id", message.session_id.as_str()),
        ("query", message.query.as_str()),
        ("camera_frame", message.camera_frame.as_str()),
        ("camera_parent_frame", message.camera_parent_frame.as_str()),
    ] {
        if value.is_empty() || value.len() > 255 {
            return Err(invalid_target(&format!(
                "{name} must contain 1..=255 bytes"
            )));
        }
    }
    if message.camera_parent_frame == message.camera_frame {
        return Err(invalid_target("camera parent and child frames must differ"));
    }
    if matches!(
        message.camera_frame.as_str(),
        "map" | "odom" | "base_link" | "base_footprint"
    ) {
        return Err(invalid_target(
            "camera calibration cannot overwrite a reserved map/odom/base child frame",
        ));
    }
    let translation = [
        message.camera_translation_x,
        message.camera_translation_y,
        message.camera_translation_z,
    ];
    let rotation = [
        message.camera_rotation_x,
        message.camera_rotation_y,
        message.camera_rotation_z,
        message.camera_rotation_w,
    ];
    if !translation.into_iter().chain(rotation).all(f64::is_finite) {
        return Err(invalid_target(
            "camera calibration contains non-finite values",
        ));
    }
    let quaternion_norm_squared = rotation.into_iter().map(|value| value * value).sum::<f64>();
    if (quaternion_norm_squared - 1.0).abs() > 1e-3 {
        return Err(invalid_target("camera quaternion is not near unit length"));
    }
    let image_width = u32::try_from(message.image_width).map_err(|_| {
        PersonFollowLcmError::InvalidConfig("target image_width is negative".into())
    })?;
    let image_height = u32::try_from(message.image_height).map_err(|_| {
        PersonFollowLcmError::InvalidConfig("target image_height is negative".into())
    })?;
    let state = match message.tracking_state {
        person_follow::PERSON_FOLLOW_TRACKED => TrackingState::Tracked,
        person_follow::PERSON_FOLLOW_LOST => TrackingState::Lost,
        person_follow::PERSON_FOLLOW_STOP => TrackingState::Stop,
        state => {
            return Err(PersonFollowLcmError::InvalidConfig(format!(
                "unknown target tracking_state {state}"
            )))
        }
    };
    let stamp = TargetStamp {
        sec: message.stamp_sec,
        nsec: message.stamp_nsec as u32,
    };
    let mut calibration = make_transform(
        message.camera_parent_frame.clone(),
        message.camera_frame.clone(),
        translation,
        rotation,
    );
    calibration.stamp_secs = stamp.sec;
    calibration.stamp_nanos = stamp.nsec;
    Ok(DecodedTarget {
        target: TrackedPerson {
            session_id: message.session_id,
            query: message.query,
            image_timestamp: stamp.sec as f64 + f64::from(stamp.nsec) * 1e-9,
            camera_frame: message.camera_frame,
            image_width,
            image_height,
            intrinsics: CameraIntrinsics::new(
                message.fx,
                message.fy,
                message.cx,
                message.cy,
                image_width,
                image_height,
            ),
            bbox: crate::BBox::new(
                message.bbox_x1,
                message.bbox_y1,
                message.bbox_x2,
                message.bbox_y2,
            ),
            state,
        },
        target_seq: message.target_seq,
        required_cloud_revision: message.required_cloud_revision,
        stamp,
        calibration,
    })
}

fn invalid_target(reason: &str) -> PersonFollowLcmError {
    PersonFollowLcmError::InvalidConfig(format!("invalid person-follow target: {reason}"))
}

fn decode_pointcloud_body(body: &[u8]) -> PersonFollowLcmResult<schema::PointCloud2> {
    let mut encoded = Vec::with_capacity(8 + body.len());
    encoded.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
    encoded.extend_from_slice(body);
    Ok(schema::decode_point_cloud2(&encoded)?)
}

async fn publish_output(
    transport: &LcmTransport,
    channel: &str,
    output: PersonFollowOutput,
) -> PersonFollowLcmResult<()> {
    match &output.state {
        PersonFollowState::Failed(reason) => {
            tracing::error!(reason, "person-follow session failed after zeroing command");
        }
        PersonFollowState::Lost => {
            tracing::warn!("person-follow target lost; session ended");
        }
        _ => {}
    }
    let state = match output.state {
        PersonFollowState::WaitingData => person_follow_command::WAITING_DATA,
        PersonFollowState::Tracking => person_follow_command::TRACKING,
        PersonFollowState::Lost => person_follow_command::LOST,
        PersonFollowState::Stopped => person_follow_command::STOPPED,
        PersonFollowState::Failed(_) => person_follow_command::FAILED,
    };
    let topic = LcmTopic::new(channel)
        .map_err(|error| PersonFollowLcmError::InvalidConfig(error.to_string()))?;
    let message_type = LcmMessageType::new(COMMAND_TYPE)
        .map_err(|error| PersonFollowLcmError::InvalidConfig(error.to_string()))?;
    let body = person_follow_command::encode_person_follow_command_body(
        &person_follow_command::PersonFollowCommand {
            session_id: output.session_id,
            target_seq: output.target_seq,
            target_stamp_sec: output.target_stamp.sec,
            target_stamp_nsec: output.target_stamp.nsec as i32,
            command_seq: output.command_seq,
            cloud_revision: output.cloud_revision,
            state,
            reason: output.reason,
            linear_x: output.command.linear[0],
            linear_y: output.command.linear[1],
            angular_z: output.command.angular[2],
        },
    )?;
    transport
        .publish(
            RawLcmFrameBuilder::new(topic, message_type)
                .payload(body)
                .build(),
        )
        .await?;
    Ok(())
}

struct TfTreeSource {
    tree: Arc<TfTree>,
}

impl TfTreeSource {
    fn new(tree: Arc<TfTree>) -> Self {
        Self { tree }
    }

    fn lookup_pose(
        &self,
        target_frame: &str,
        source_frame: &str,
        query: Option<(f64, f64)>,
    ) -> Option<PoseStamped> {
        let transform = match query {
            Some((time_point, time_tolerance)) => self
                .tree
                .chain_at(source_frame, target_frame, time_point, time_tolerance)
                .ok()?,
            None => self.tree.chain(source_frame, target_frame).ok()?,
        };
        let translation = transform.translation.vector;
        let rotation = transform.rotation.quaternion();
        Some(PoseStamped {
            frame_id: target_frame.into(),
            position: [translation.x, translation.y, translation.z],
            orientation: [rotation.i, rotation.j, rotation.k, rotation.w],
        })
    }
}

impl TfLike for TfTreeSource {
    fn lookup(&self, target_frame: &str, source_frame: &str) -> Option<PoseStamped> {
        self.lookup_pose(target_frame, source_frame, None)
    }

    fn lookup_at(
        &self,
        target_frame: &str,
        source_frame: &str,
        time_point: Option<f64>,
        time_tolerance: f64,
    ) -> Option<PoseStamped> {
        self.lookup_pose(
            target_frame,
            source_frame,
            time_point.map(|time_point| (time_point, time_tolerance)),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn target_message() -> person_follow::PersonFollowTarget {
        person_follow::PersonFollowTarget {
            session_id: "session".into(),
            query: "person".into(),
            target_seq: 4,
            required_cloud_revision: 7,
            stamp_sec: 1_000_000_000,
            stamp_nsec: 123,
            camera_frame: "camera_optical".into(),
            camera_parent_frame: "base_link".into(),
            camera_translation_x: 0.1,
            camera_translation_y: 0.2,
            camera_translation_z: 0.3,
            camera_rotation_x: 0.0,
            camera_rotation_y: 0.0,
            camera_rotation_z: 0.0,
            camera_rotation_w: 1.0,
            image_width: 640,
            image_height: 480,
            fx: 400.0,
            fy: 400.0,
            cx: 320.0,
            cy: 240.0,
            bbox_x1: 100.0,
            bbox_y1: 100.0,
            bbox_x2: 200.0,
            bbox_y2: 300.0,
            tracking_state: person_follow::PERSON_FOLLOW_TRACKED,
        }
    }

    #[test]
    fn default_command_channel_is_gateway_guard_ingress() {
        let config = PersonFollowLcmConfig::default();
        assert_eq!(config.command_channel, "/gateway/person_follow_cmd_vel");
        validate_config(&config).expect("default person-follow LCM config");
    }

    #[test]
    fn direct_and_legacy_motion_channels_are_rejected() {
        for forbidden in ["/cmd_vel", "/nav_cmd_vel"] {
            let config = PersonFollowLcmConfig {
                command_channel: forbidden.into(),
                ..PersonFollowLcmConfig::default()
            };
            let error = validate_config(&config).expect_err("forbidden motion channel");
            assert!(error.to_string().contains("forbidden"));
        }
    }

    #[test]
    fn target_decode_preserves_exact_stamp_and_local_calibration() {
        let body = person_follow::encode_person_follow_target_body(&target_message())
            .expect("encode target");
        let decoded = decode_target(&body).expect("decode target");

        assert_eq!(decoded.target_seq, 4);
        assert_eq!(decoded.required_cloud_revision, 7);
        assert_eq!(decoded.stamp.sec, 1_000_000_000);
        assert_eq!(decoded.stamp.nsec, 123);
        assert_eq!(decoded.calibration.parent, "base_link");
        assert_eq!(decoded.calibration.child, "camera_optical");
        assert_eq!(decoded.calibration.stamp_secs, 1_000_000_000);
        assert_eq!(decoded.calibration.stamp_nanos, 123);
        assert_eq!(decoded.calibration.transform.translation.vector.x, 0.1);
    }

    #[test]
    fn target_query_is_bounded_to_255_bytes() {
        let mut accepted = target_message();
        accepted.query = "q".repeat(255);
        let body = person_follow::encode_person_follow_target_body(&accepted)
            .expect("encode bounded target");
        decode_target(&body).expect("255-byte query");

        let mut rejected = target_message();
        rejected.query = "q".repeat(256);
        let body = person_follow::encode_person_follow_target_body(&rejected)
            .expect("encode overlong target");
        assert!(decode_target(&body)
            .expect_err("256-byte query")
            .to_string()
            .contains("1..=255 bytes"));
    }

    #[test]
    fn invalid_calibration_and_reserved_child_are_rejected() {
        let mut message = target_message();
        message.camera_rotation_w = 0.5;
        let body =
            person_follow::encode_person_follow_target_body(&message).expect("encode target");
        assert!(decode_target(&body)
            .expect_err("non-unit quaternion")
            .to_string()
            .contains("near unit"));

        let mut message = target_message();
        message.camera_frame = "odom".into();
        let body =
            person_follow::encode_person_follow_target_body(&message).expect("encode target");
        assert!(decode_target(&body)
            .expect_err("reserved child")
            .to_string()
            .contains("reserved"));
    }

    #[tokio::test]
    async fn ready_callback_runs_after_real_lcm_subscriptions() {
        let transport = LcmTransport::new_real("memq://").expect("real LCM memq context");
        let owner = PersonFollowLcmOwner::new(transport, PersonFollowLcmConfig::default())
            .expect("person-follow owner");
        let (shutdown_tx, shutdown_rx) = tokio::sync::oneshot::channel();
        let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();
        let task = tokio::spawn(owner.run_until_with_ready(
            async move {
                let _ = shutdown_rx.await;
            },
            move || {
                let _ = ready_tx.send(());
                Ok(())
            },
        ));

        tokio::time::timeout(std::time::Duration::from_secs(1), ready_rx)
            .await
            .expect("ready callback timeout")
            .expect("ready callback sender");
        shutdown_tx.send(()).expect("request shutdown");
        task.await
            .expect("owner task joins")
            .expect("owner stops cleanly");
    }
}
