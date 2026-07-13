//! Real-LCM publisher/subscriber service for [`crate::TfTree`].

use std::sync::Arc;

use bytes::Bytes;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use nuwax_codec::lcm::schema;
use nuwax_lcm::{LcmContext, LcmError, LcmSubscription};
use thiserror::Error;
use tokio::sync::oneshot;
use tokio::task::JoinHandle;

use crate::{TfTree, TransformStamped};

/// Locked typed channel used by the DimOS TF service.
pub const DEFAULT_TF_CHANNEL: &str = "/tf#tf2_msgs.TFMessage";
const TF_MESSAGE_TYPE: &str = "tf2_msgs.TFMessage";

/// Errors from the real-LCM TF service.
#[derive(Debug, Error)]
pub enum LcmTfError {
    /// liblcm lifecycle or publication failure.
    #[error(transparent)]
    Lcm(#[from] LcmError),
    /// Generated TFMessage codec failure.
    #[error(transparent)]
    Codec(#[from] nuwax_codec::CodecError),
    /// Channel does not carry the locked TFMessage typed suffix.
    #[error("TF channel '{0}' must use #tf2_msgs.TFMessage")]
    InvalidTypedChannel(String),
    /// Timestamp seconds cannot be represented by the generated schema.
    #[error("TF timestamp seconds {0} exceed generated int32 range")]
    TimestampOutOfRange(i64),
    /// Nanoseconds are outside the normalized ROS interval.
    #[error("TF timestamp nanoseconds {0} must be less than 1,000,000,000")]
    InvalidNanoseconds(u32),
    /// Decoded transform contains non-finite or zero-norm geometry.
    #[error("invalid transform geometry for {0}")]
    InvalidTransform(String),
    /// The callback reported a different channel than the exact subscription.
    #[error("TF callback channel mismatch: expected '{expected}', got '{actual}'")]
    ChannelMismatch {
        /// Configured subscription channel.
        expected: String,
        /// Channel supplied by the liblcm callback.
        actual: String,
    },
    /// Rust-side delivery queue lagged.
    #[error("TF subscription queue lagged by {0} messages")]
    SubscriptionLagged(u64),
    /// Subscription ended without an explicit service stop.
    #[error("TF subscription closed unexpectedly")]
    SubscriptionClosed,
    /// Tokio receiver task failed.
    #[error("TF receiver task failed: {0}")]
    ReceiverTask(String),
}

/// Result alias for the real-LCM TF service.
pub type LcmTfResult<T> = Result<T, LcmTfError>;

/// A running typed `/tf` publisher/subscriber backed by one [`LcmContext`].
///
/// Use [`Self::from_provider`] when the service owns its LCM context. Use
/// [`Self::from_context`] to share one context with other runtime adapters; the
/// caller then remains responsible for shutting that context down.
pub struct LcmTf {
    tree: Arc<TfTree>,
    context: Arc<LcmContext>,
    channel: String,
    shutdown: Option<oneshot::Sender<()>>,
    receiver: Option<JoinHandle<LcmTfResult<()>>>,
    owns_context: bool,
}

impl LcmTf {
    /// Start a TF service with its own real liblcm context.
    pub async fn from_provider(
        provider: &str,
        channel: &str,
        buffer_size_secs: f64,
    ) -> LcmTfResult<Self> {
        let context = Arc::new(LcmContext::new(provider)?);
        Self::start(context, channel, buffer_size_secs, true).await
    }

    /// Start a TF service on an existing real liblcm context.
    pub async fn from_context(
        context: Arc<LcmContext>,
        channel: &str,
        buffer_size_secs: f64,
    ) -> LcmTfResult<Self> {
        Self::start(context, channel, buffer_size_secs, false).await
    }

    async fn start(
        context: Arc<LcmContext>,
        channel: &str,
        buffer_size_secs: f64,
        owns_context: bool,
    ) -> LcmTfResult<Self> {
        validate_typed_channel(channel)?;
        let subscription = context.subscribe(channel).await?;
        let tree = Arc::new(TfTree::with_buffer_size(buffer_size_secs));
        let receiver_tree = Arc::clone(&tree);
        let expected_channel = channel.to_string();
        let (shutdown_tx, shutdown_rx) = oneshot::channel();
        let receiver = tokio::spawn(run_receiver(
            subscription,
            receiver_tree,
            expected_channel,
            shutdown_rx,
        ));
        Ok(Self {
            tree,
            context,
            channel: channel.to_string(),
            shutdown: Some(shutdown_tx),
            receiver: Some(receiver),
            owns_context,
        })
    }

    /// Return the live transform tree populated by local and received frames.
    #[must_use]
    pub fn tree(&self) -> Arc<TfTree> {
        Arc::clone(&self.tree)
    }

    /// Return the exact typed LCM channel.
    #[must_use]
    pub fn channel(&self) -> &str {
        &self.channel
    }

    /// Publish transforms and insert them into the local tree first.
    ///
    /// This matches the original `PubSubTF.publish` ordering: local queries can
    /// observe the transform even when the transport publication subsequently
    /// fails, while the publish error remains visible to the caller.
    pub async fn publish(&self, transforms: &[TransformStamped]) -> LcmTfResult<()> {
        for transform in transforms {
            self.tree.set(transform.clone());
        }
        let payload = encode_tf_message(transforms)?;
        self.context
            .publish(&self.channel, Bytes::from(payload))
            .await?;
        Ok(())
    }

    /// Confirm subscription teardown and, for an owned context, owner-thread shutdown.
    pub async fn stop(mut self) -> LcmTfResult<()> {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
        let receiver_result = match self.receiver.take() {
            Some(receiver) => receiver
                .await
                .map_err(|error| LcmTfError::ReceiverTask(error.to_string()))?,
            None => Ok(()),
        };
        let context_result = if self.owns_context {
            self.context.shutdown().await.map_err(LcmTfError::from)
        } else {
            Ok(())
        };
        receiver_result?;
        context_result
    }
}

impl Drop for LcmTf {
    fn drop(&mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
        if let Some(receiver) = self.receiver.take() {
            receiver.abort();
        }
    }
}

async fn run_receiver(
    mut subscription: LcmSubscription,
    tree: Arc<TfTree>,
    expected_channel: String,
    mut shutdown: oneshot::Receiver<()>,
) -> LcmTfResult<()> {
    loop {
        tokio::select! {
            _ = &mut shutdown => {
                subscription.unsubscribe().await?;
                return Ok(());
            }
            received = subscription.receiver.recv() => {
                let received = match received {
                    Ok(received) => received,
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(count)) => {
                        return Err(LcmTfError::SubscriptionLagged(count));
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                        return Err(LcmTfError::SubscriptionClosed);
                    }
                };
                if received.channel != expected_channel {
                    return Err(LcmTfError::ChannelMismatch {
                        expected: expected_channel,
                        actual: received.channel,
                    });
                }
                for transform in decode_tf_message(&received.payload)? {
                    tree.set(transform);
                }
            }
        }
    }
}

/// Encode native transforms as a complete generated `tf2_msgs.TFMessage`.
pub fn encode_tf_message(transforms: &[TransformStamped]) -> LcmTfResult<Vec<u8>> {
    let transforms = transforms
        .iter()
        .map(to_schema_transform)
        .collect::<LcmTfResult<Vec<_>>>()?;
    Ok(schema::encode_tf_message(&schema::TfMessage {
        transforms,
    })?)
}

/// Decode a complete generated `tf2_msgs.TFMessage` into native transforms.
pub fn decode_tf_message(encoded: &[u8]) -> LcmTfResult<Vec<TransformStamped>> {
    schema::decode_tf_message(encoded)?
        .transforms
        .into_iter()
        .map(from_schema_transform)
        .collect()
}

fn validate_typed_channel(channel: &str) -> LcmTfResult<()> {
    if channel
        .rsplit_once('#')
        .is_some_and(|(topic, message_type)| !topic.is_empty() && message_type == TF_MESSAGE_TYPE)
    {
        Ok(())
    } else {
        Err(LcmTfError::InvalidTypedChannel(channel.to_string()))
    }
}

fn to_schema_transform(transform: &TransformStamped) -> LcmTfResult<schema::TransformStamped> {
    let stamp_sec = i32::try_from(transform.stamp_secs)
        .map_err(|_| LcmTfError::TimestampOutOfRange(transform.stamp_secs))?;
    if transform.stamp_nanos >= 1_000_000_000 {
        return Err(LcmTfError::InvalidNanoseconds(transform.stamp_nanos));
    }
    let rotation = transform.transform.rotation;
    Ok(schema::TransformStamped {
        header: schema::Header {
            seq: 0,
            stamp_sec,
            stamp_nsec: transform.stamp_nanos as i32,
            frame_id: transform.parent.clone(),
        },
        child_frame_id: transform.child.clone(),
        transform: schema::Transform {
            translation: [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            rotation_xyzw: [rotation.i, rotation.j, rotation.k, rotation.w],
        },
    })
}

fn from_schema_transform(transform: schema::TransformStamped) -> LcmTfResult<TransformStamped> {
    let [x, y, z] = transform.transform.translation;
    let [qx, qy, qz, qw] = transform.transform.rotation_xyzw;
    if transform.header.stamp_nsec < 0 || transform.header.stamp_nsec >= 1_000_000_000 {
        return Err(LcmTfError::InvalidNanoseconds(
            transform.header.stamp_nsec.max(0) as u32,
        ));
    }
    if ![x, y, z, qx, qy, qz, qw].into_iter().all(f64::is_finite) {
        return Err(LcmTfError::InvalidTransform(format!(
            "{} -> {}",
            transform.header.frame_id, transform.child_frame_id
        )));
    }
    let quaternion = Quaternion::new(qw, qx, qy, qz);
    if quaternion.norm_squared() <= f64::EPSILON {
        return Err(LcmTfError::InvalidTransform(format!(
            "{} -> {}",
            transform.header.frame_id, transform.child_frame_id
        )));
    }
    Ok(TransformStamped {
        parent: transform.header.frame_id,
        child: transform.child_frame_id,
        stamp_secs: i64::from(transform.header.stamp_sec),
        stamp_nanos: transform.header.stamp_nsec as u32,
        transform: Isometry3::from_parts(
            Translation3::new(x, y, z),
            UnitQuaternion::new_normalize(quaternion),
        ),
    })
}
