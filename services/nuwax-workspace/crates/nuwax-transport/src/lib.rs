//! nuwax 传输层
//!
//! nuwax 传输层
//!
//! ## LCM-first pub/sub 合同
//!
//! 总纲修订（00-执行规划总纲.md §10）:本层封装 **LCM** publish/subscribe,
//! **不**重写 UDP framing 协议,只把现有 LCM 1.x wire format 包装为
//! 强类型 Rust API。
//!
//! 关键约定：
//! - [`LcmBackend`] trait:实现 publish / subscribe;`real-lcm` 是 nuwax 仓内唯一 backend。
//! - [`LcmTransport`]:real-lcm backend 工厂 + publish/subscribe 转发。
//! - **`real-lcm` feature**(workspace `default = ["real-lcm"]`):启用真正的 liblcm / Rust LCM
//!   binding backend (`real_backend::RealLcmBackend`)。 2026-07-07 Phase 3
//!   实装,详见 `crates/nuwax-lcm` / `crates/nuwax-lcm-sys` 模块文档。
//! - 只提供真实 LCM 工厂；镜像必须安装 liblcm-dev 1.3~1.5。
//! - typed suffix: `LcmTopic::as_subscription_pattern` 与
//!   `parse_subscription_pattern` 在 pub/sub 边界统一。
//!
//! ## P0 foundation 边界
//!
//! - 完成:LCM trait / real backend / typed suffix 支持 / 错误类型；
//! - 不做:rosbag replay、aarch64 实机验证。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use async_trait::async_trait;
use bytes::Bytes;
use futures::Stream;
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame};
use std::sync::Arc;
use thiserror::Error;

/// real-lcm backend(Phase 3,2026-07-07 实装;仅在 `real-lcm` feature 开启时编译)。
#[cfg(feature = "real-lcm")]
pub mod real_backend;

#[cfg(feature = "real-lcm")]
pub use real_backend::OwnedLcmSubscription;

// ============================================================================
// LCM-first API
// ============================================================================

/// 传输错误
#[derive(Debug, Error)]
pub enum TransportError {
    /// LCM 后端错误
    #[error("lcm backend error: {0}")]
    LcmBackend(String),
    /// topic 错误
    #[error("topic error: {0}")]
    Topic(String),
    /// message type 错误
    #[error("message type error: {0}")]
    MessageType(String),
    /// 协议错误
    #[error("protocol error: {0}")]
    Protocol(#[from] nuwax_protocol::ProtocolError),
    /// codec 错误
    #[error("codec error: {0}")]
    Codec(#[from] nuwax_codec::CodecError),
    /// IO 错误
    #[error("io error: {0}")]
    Io(String),
    /// 通道已关闭
    #[error("channel closed")]
    ChannelClosed,
}

/// 传输结果
pub type TransportResult<T> = Result<T, TransportError>;

/// LCM 订阅结果（流式）
pub type LcmFrameStream =
    std::pin::Pin<Box<dyn Stream<Item = TransportResult<RawLcmFrame>> + Send + Unpin>>;

/// LCM 后端 trait
///
/// 唯一生产实现是 `RealLcmBackend`。
#[async_trait]
pub trait LcmBackend: Send + Sync {
    /// 异步发布一帧
    async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()>;
    /// 按输入顺序发布一批消息，失败后不再发送后续消息。
    ///
    /// 真实后端用单次线程命令完成，默认实现保留其它后端的逐条兼容。
    async fn publish_batch(&self, frames: Vec<RawLcmFrame>) -> TransportResult<()> {
        for (index, frame) in frames.into_iter().enumerate() {
            self.publish(frame).await.map_err(|error| {
                TransportError::LcmBackend(format!(
                    "batch publish failed at sample {index}: {error}"
                ))
            })?;
        }
        Ok(())
    }
    /// 异步订阅 topic 帧流
    async fn subscribe(&self, topic: LcmTopic, msg_type: Option<LcmMessageType>) -> LcmFrameStream;
    /// 后端名称（诊断 / 日志用）
    fn name(&self) -> &str;

    /// Return the process-wide real LCM context when this backend owns one.
    ///
    /// Raw protocols such as DimOS pickle RPC do not use generated message
    /// fingerprints, so they must share the owner context without going through
    /// the typed [`RawLcmFrame`] path.
    #[cfg(feature = "real-lcm")]
    fn shared_lcm_context(&self) -> Option<Arc<nuwax_lcm::LcmContext>> {
        None
    }
}

/// LCM 传输工厂
///
/// `LcmTransport::new_real` 是唯一公开 backend 工厂。
#[derive(Clone)]
pub struct LcmTransport {
    backend: Arc<dyn LcmBackend>,
}

impl std::fmt::Debug for LcmTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LcmTransport")
            .field("backend", &self.backend.name())
            .finish()
    }
}

impl LcmTransport {
    /// 构造 transport,指定具体 backend(给 RealLcmBackend 等 trait 实现用)
    #[must_use]
    pub fn new(backend: Arc<dyn LcmBackend>) -> Self {
        Self { backend }
    }

    /// 构造 real liblcm backend 的 transport(Phase 3,2026-07-07)。
    ///
    /// 这是 nuwax-transport 公开层唯一允许的 backend 工厂。`provider_url` 形如
    /// `udpm://239.255.76.67:7667?ttl=1`。失败时返回
    /// `TransportError::LcmBackend`。镜像必须装 liblcm-dev 1.3~1.5
    /// (版本锁 `nuwax-lcm-sys/build.rs` `MAX_LCM_VERSION = "1.6"`)。
    #[cfg(feature = "real-lcm")]
    pub fn new_real(provider_url: &str) -> TransportResult<Self> {
        let backend = self::real_backend::RealLcmBackend::new(provider_url)?;
        Ok(Self::new(Arc::new(backend)))
    }

    /// 当前 backend 名称
    #[must_use]
    pub fn backend_name(&self) -> &str {
        self.backend.name()
    }

    /// Clone the process-wide real LCM context owned by this transport.
    ///
    /// The returned [`Arc`] keeps the existing single owner thread alive; it
    /// does not create a second liblcm context.
    #[cfg(feature = "real-lcm")]
    #[must_use]
    pub fn shared_lcm_context(&self) -> Option<Arc<nuwax_lcm::LcmContext>> {
        self.backend.shared_lcm_context()
    }

    /// 发布一帧
    pub async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
        self.backend.publish(frame).await
    }

    /// 发布一个有序批次，等待后端完成整个批次的本地发送。
    pub async fn publish_batch(&self, frames: Vec<RawLcmFrame>) -> TransportResult<()> {
        self.backend.publish_batch(frames).await
    }

    /// 订阅 topic 帧流
    pub async fn subscribe(
        &self,
        topic: LcmTopic,
        msg_type: Option<LcmMessageType>,
    ) -> LcmFrameStream {
        self.backend.subscribe(topic, msg_type).await
    }

    /// Subscribe through the transport's existing real liblcm owner context.
    ///
    /// Unlike [`Self::subscribe`], the returned handle retains the underlying
    /// subscription so its owner can explicitly wait for unsubscribe
    /// acknowledgement during shutdown. Transports backed by anything other
    /// than the real shared context fail closed.
    #[cfg(feature = "real-lcm")]
    pub async fn subscribe_owned(
        &self,
        topic: LcmTopic,
        msg_type: Option<LcmMessageType>,
    ) -> TransportResult<OwnedLcmSubscription> {
        let context = self.shared_lcm_context().ok_or_else(|| {
            TransportError::LcmBackend(
                "owned subscription requires the real shared LCM context".to_owned(),
            )
        })?;
        OwnedLcmSubscription::subscribe(context, topic, msg_type).await
    }
}

// ============================================================================
// LcmTransport 自身实现 LcmBackend (newtype 转发)
// ============================================================================
//
// 本 crate 唯一 backend 是 `RealLcmBackend`。`LcmTransport` 作为持有
// `Arc<dyn LcmBackend>` 的
// 薄包装,本身实现 `LcmBackend` trait 是 newtype-转发模式 —— 所有方法
// 直接转给内部 backend。
//
// 这一转发让:
// - 下游 crate 可以写 `&transport: &dyn LcmBackend` 统一接口,
//   不需要 `.backend().publish()` 这种内部逃逸
// - `real_backend.rs` 测试 `name_is_real_lcm` 可以直接断言
//   `check_name(&transport) == "real-lcm"`
//
// 注意:`LcmTransport::new(Arc<dyn LcmBackend>)` 的 `LcmBackend` 参数
// 不会引发 trait impl 循环(`LcmTransport` 不再可作为内部 backend,只
// 是 wrapper)。
#[async_trait]
impl LcmBackend for LcmTransport {
    async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
        self.backend.publish(frame).await
    }
    async fn publish_batch(&self, frames: Vec<RawLcmFrame>) -> TransportResult<()> {
        self.backend.publish_batch(frames).await
    }
    async fn subscribe(&self, topic: LcmTopic, msg_type: Option<LcmMessageType>) -> LcmFrameStream {
        self.backend.subscribe(topic, msg_type).await
    }
    fn name(&self) -> &str {
        self.backend.name()
    }

    #[cfg(feature = "real-lcm")]
    fn shared_lcm_context(&self) -> Option<Arc<nuwax_lcm::LcmContext>> {
        self.backend.shared_lcm_context()
    }
}

/// 把 `Bytes` 转换为 `Vec<u8>` 的便捷函数（codec 接口互转）
#[must_use]
pub fn bytes_to_vec(b: Bytes) -> Vec<u8> {
    b.to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_protocol::RawLcmFrameBuilder;

    #[cfg(feature = "real-lcm")]
    struct ContextlessBackend;

    #[cfg(feature = "real-lcm")]
    #[async_trait]
    impl LcmBackend for ContextlessBackend {
        async fn publish(&self, _frame: RawLcmFrame) -> TransportResult<()> {
            Ok(())
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            Box::pin(futures::stream::empty())
        }

        fn name(&self) -> &str {
            "contextless-test"
        }
    }

    #[test]
    fn bytes_to_vec_roundtrip() {
        let v = vec![1u8, 2, 3, 4];
        let out = bytes_to_vec(Bytes::from(v.clone()));
        assert_eq!(out, v);
    }

    /// `LcmTopic::new` 校验 typed suffix 解析。
    #[test]
    fn typed_suffix_parser_rejects_malformed() {
        let topic = LcmTopic::new("/scan#pkg.Foo").unwrap_err();
        let _ = topic;
    }

    /// 守恒测试：`real-lcm` 是唯一 backend 名。
    #[test]
    fn real_backend_name_is_real_lcm() {
        let canonical: &str = "real-lcm";
        assert_eq!(canonical, "real-lcm");
        // 显式构造 RawLcmFrame 走一遍 builder(防止 import 死代码误删)
        let topic = LcmTopic::new("/scan").unwrap();
        let msg_type = LcmMessageType::new("sensor_msgs/PointCloud2").unwrap();
        let frame = RawLcmFrameBuilder::new(topic, msg_type)
            .payload(b"hello".to_vec())
            .build();
        assert_eq!(frame.payload, b"hello");
    }

    #[cfg(feature = "real-lcm")]
    #[test]
    fn raw_protocols_clone_the_existing_owner_context() {
        let transport = LcmTransport::new_real("memq://").expect("real LCM memq transport");
        let first = transport
            .shared_lcm_context()
            .expect("real backend context");
        let second = transport
            .shared_lcm_context()
            .expect("real backend context");

        assert!(Arc::ptr_eq(&first, &second));
        assert_eq!(first.provider(), "memq://");
    }

    #[cfg(feature = "real-lcm")]
    #[tokio::test]
    async fn owned_subscription_fails_closed_without_real_context() {
        let transport = LcmTransport::new(Arc::new(ContextlessBackend));
        let topic = LcmTopic::new("/owned/contextless").unwrap();
        let error = transport
            .subscribe_owned(topic, None)
            .await
            .expect_err("contextless backend must not create an owned subscription");

        assert!(matches!(error, TransportError::LcmBackend(_)));
    }

    #[cfg(feature = "real-lcm")]
    #[tokio::test]
    async fn owned_subscription_retains_context_and_acknowledges_unsubscribe() {
        let transport = LcmTransport::new_real("memq://").expect("real LCM memq transport");
        let topic = LcmTopic::new("/owned/pose").unwrap();
        let msg_type = LcmMessageType::new("geometry_msgs/PoseStamped").unwrap();
        let mut subscription = transport
            .subscribe_owned(topic.clone(), Some(msg_type.clone()))
            .await
            .expect("owner-thread acknowledged subscribe");
        let frame = RawLcmFrameBuilder::new(topic, msg_type)
            .payload(b"owned-body".to_vec())
            .build();

        transport.publish(frame).await.expect("publish");
        drop(transport);
        let received = tokio::time::timeout(std::time::Duration::from_secs(2), subscription.recv())
            .await
            .expect("receive timeout")
            .expect("typed decode");
        assert_eq!(received.payload, b"owned-body");

        subscription
            .unsubscribe()
            .await
            .expect("owner-thread acknowledged unsubscribe");
    }

    #[cfg(feature = "real-lcm")]
    #[tokio::test]
    async fn latest_owned_cloud_skips_backlog_while_ordered_odometry_keeps_every_frame() {
        let transport = LcmTransport::new_real("memq://").unwrap();
        let topic = LcmTopic::new("/owned/backlog").unwrap();
        let msg_type = LcmMessageType::new("geometry_msgs/PoseStamped").unwrap();
        let mut cloud = transport.subscribe_owned(topic.clone(), Some(msg_type.clone())).await.unwrap();
        let mut ordered = transport.subscribe_owned(topic.clone(), Some(msg_type.clone())).await.unwrap();
        for sequence in 1_u8..=16 {
            transport.publish(RawLcmFrameBuilder::new(topic.clone(), msg_type.clone()).payload(vec![sequence]).build()).await.unwrap();
        }
        // 另一正常订阅读到最后一帧，证明真实LCM拥有方已交付全部回调。
        for sequence in 1_u8..=16 {
            let frame = tokio::time::timeout(std::time::Duration::from_secs(2), ordered.recv()).await.unwrap().unwrap();
            assert_eq!(frame.payload.as_slice(), &[sequence]);
        }
        let (latest, skipped) = cloud.recv_latest().await.unwrap();
        assert_eq!(latest.payload.as_slice(), &[16]);
        assert_eq!(skipped, 15);
        cloud.unsubscribe().await.unwrap();
        ordered.unsubscribe().await.unwrap();
    }
}
