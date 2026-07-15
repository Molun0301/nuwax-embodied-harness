//! Real-LCM production owner for the person-follow lifecycle.

use std::future::Future;
use std::sync::Arc;
use std::time::Instant;

use futures::StreamExt;
use nuwax_codec::lcm::{person_follow, schema};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame, RawLcmFrameBuilder};
use nuwax_tf::lcm::{LcmTf, LcmTfError};
use nuwax_tf::TfTree;
use nuwax_transport::{LcmFrameStream, LcmTransport, TransportError};
use thiserror::Error;

use crate::person_follow::{
    PackedPointCloud, PersonFollowConfig, PersonFollowController, PersonFollowError,
    PersonFollowOutput, TrackedPerson, TrackingState,
};
use crate::{CameraIntrinsics, DetectionNavigationConfig, PoseStamped, TfLike, Twist};

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
const TWIST_TYPE: &str = "geometry_msgs.Twist";
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
        let controller = PersonFollowController::new(
            Box::new(TfTreeSource::new(tf.tree())),
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
                self.run_loop(controller, targets, pointclouds, shutdown)
                    .await
            }
            Err(error) => Err(error),
        };
        let zero_result = publish_zero(&self.transport, &self.config.command_channel).await;
        let tf_result = tf.stop().await.map_err(PersonFollowLcmError::from);
        runtime_result?;
        zero_result?;
        tf_result
    }

    async fn run_loop<F>(
        &self,
        mut controller: PersonFollowController,
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
                    let target = decode_target(&frame.payload)?;
                    controller.update_target(target, started.elapsed());
                    if let Some(output) = controller.tick(started.elapsed()) {
                        publish_output(&self.transport, &self.config.command_channel, output).await?;
                    }
                }
                frame = pointclouds.next() => {
                    let frame = required_frame(frame, "person-follow pointcloud")?;
                    let cloud = decode_pointcloud_body(&frame.payload)?;
                    controller.update_pointcloud(PackedPointCloud::from_lcm(&cloud)?);
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

fn decode_target(body: &[u8]) -> PersonFollowLcmResult<TrackedPerson> {
    let message = person_follow::decode_person_follow_target_body(body)?;
    if !(0..1_000_000_000).contains(&message.stamp_nsec) {
        return Err(PersonFollowLcmError::InvalidConfig(format!(
            "target stamp_nsec {} is outside [0, 1e9)",
            message.stamp_nsec
        )));
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
    Ok(TrackedPerson {
        session_id: message.session_id,
        query: message.query,
        image_timestamp: message.stamp_sec as f64 + f64::from(message.stamp_nsec) * 1e-9,
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
    })
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
        crate::person_follow::PersonFollowState::Failed(reason) => {
            tracing::error!(reason, "person-follow session failed after zeroing command");
        }
        crate::person_follow::PersonFollowState::Lost {
            frames,
            terminal: true,
        } => {
            tracing::warn!(frames, "person-follow target lost; session ended");
        }
        _ => {}
    }
    publish_twist(transport, channel, output.command).await
}

async fn publish_zero(transport: &LcmTransport, channel: &str) -> PersonFollowLcmResult<()> {
    publish_twist(transport, channel, Twist::zero()).await
}

async fn publish_twist(
    transport: &LcmTransport,
    channel: &str,
    command: Twist,
) -> PersonFollowLcmResult<()> {
    let topic = LcmTopic::new(channel)
        .map_err(|error| PersonFollowLcmError::InvalidConfig(error.to_string()))?;
    let message_type = LcmMessageType::new(TWIST_TYPE)
        .map_err(|error| PersonFollowLcmError::InvalidConfig(error.to_string()))?;
    let body = schema::encode_twist_body(&schema::Twist {
        linear: command.linear,
        angular: command.angular,
    });
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
