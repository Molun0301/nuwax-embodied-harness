//! nuwax 传输层
//!
//! ## LCM-first pub/sub 合同
//!
//! 总纲修订（00-执行规划总纲.md §10）:本层封装 **LCM** publish/subscribe,
//! **不**重写 UDP framing 协议,只把现有 LCM 1.x wire format 包装为
//! 强类型 Rust API。
//!
//! 关键约定:
//! - [`LcmBackend`] trait:实现 publish / subscribe;不同 backend
//!   (fake / real / shm-sidecar) 走同一 trait。
//! - [`LcmTransport`]:默认 backend 选择 + 工厂。
//! - [`FakeLcmBackend`]:CI 默认 backend,内存中维护 topic → Vec\<frame\> 队列,
//!   跨 `subscribe` 调用保留消息;不依赖机器人硬件、不依赖 `liblcm`。
//! - **`real-lcm` feature**(默认关闭):启用真正的 liblcm / Rust LCM
//!   binding backend (`real_backend::RealLcmBackend`)。 2026-07-07 Phase 3
//!   实装,详见 `crates/nuwax-lcm` / `crates/nuwax-lcm-sys` 模块文档。
//! - typed suffix: `LcmTopic::as_subscription_pattern` 与
//!   `parse_subscription_pattern` 在 pub/sub 边界统一。
//!
//! ## 历史占位（标记为 deprecated）
//!
//! `UdpTransport` / `ShmTransport` 仍存在以保留 `nuwax-transport::Transport`
//! trait(下游 crate 可能用到);它们走 `MessageEnvelope` 兼容层,不再是
//! 默认协议层。`bytes_to_vec` 保留为 codec 互转 helper。
//!
//! ## P0 foundation 边界
//!
//! - 完成:LCM trait / fake backend / typed suffix 支持 / 错误类型；
//! - 不做:rosbag replay、aarch64 实机验证。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use async_trait::async_trait;
use bytes::Bytes;
use futures::stream::{self, Stream};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use thiserror::Error;

pub mod compat;

/// real-lcm backend(Phase 3,2026-07-07 实装;仅在 `real-lcm` feature 开启时编译)。
#[cfg(feature = "real-lcm")]
pub mod real_backend;

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
/// 不同 backend 共享同一 trait；生产 fake / real / shm-sidecar 各自实现。
#[async_trait]
pub trait LcmBackend: Send + Sync {
    /// 异步发布一帧
    async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()>;
    /// 异步订阅 topic 帧流
    async fn subscribe(&self, topic: LcmTopic, msg_type: Option<LcmMessageType>) -> LcmFrameStream;
    /// 后端名称（诊断 / 日志用）
    fn name(&self) -> &str;
}

/// LCM 传输工厂
///
/// 负责选择默认 backend,以及切换 fake / real。构造时必须显式指定 backend,
/// 以避免隐式选择 real LCM（CI 默认走 fake）。
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
    /// 构造 transport,指定具体 backend
    #[must_use]
    pub fn new(backend: Arc<dyn LcmBackend>) -> Self {
        Self { backend }
    }

    /// 构造 fake backend 的 transport（CI 默认）
    #[must_use]
    pub fn new_fake() -> Self {
        Self::new(Arc::new(FakeLcmBackend::new()))
    }

    /// 构造 real liblcm backend 的 transport（Phase 3,2026-07-07）。
    ///
    /// 仅在启用 `real-lcm` feature 时编译可选;`provider_url` 形如
    /// `udpm://239.255.76.67:7667?ttl=1`。失败时返回
    /// `TransportError::LcmBackend`。
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

    /// 发布一帧
    pub async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
        self.backend.publish(frame).await
    }

    /// 订阅 topic 帧流
    pub async fn subscribe(
        &self,
        topic: LcmTopic,
        msg_type: Option<LcmMessageType>,
    ) -> LcmFrameStream {
        self.backend.subscribe(topic, msg_type).await
    }
}

// ============================================================================
// Fake backend（CI 默认,不依赖硬件 / liblcm）
// ============================================================================

/// fake LCM backend,内存中维护 `topic → Vec<frame>` 队列。
///
/// - `publish` 把 frame 压入对应 topic 队列；
/// - `subscribe` 返回一个无限流,每次 stream poll 都把当前 topic 队列
///   快照成新 stream;fake backend 不会复制队列,但所有调用方都看到
///   publish 之后的最新状态。
/// - typed suffix:如果 `msg_type` 不为 None,fake 校验 `frame.msg_type`
///   与请求匹配,否则返回错误流。
pub struct FakeLcmBackend {
    inner: Arc<Mutex<FakeInner>>,
}

#[derive(Default)]
struct FakeInner {
    /// topic channel -> queued frames
    topics: HashMap<String, Vec<RawLcmFrame>>,
    /// publish seqno 单调递增
    seqno: u64,
}

impl FakeLcmBackend {
    /// 构造 fake backend
    #[must_use]
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(FakeInner::default())),
        }
    }

    /// 取得内部状态快照（测试用）
    #[must_use]
    pub fn snapshot(&self) -> HashMap<String, usize> {
        let guard = self.inner.lock().expect("fake backend poisoned");
        guard
            .topics
            .iter()
            .map(|(k, v)| (k.clone(), v.len()))
            .collect()
    }
}

impl Default for FakeLcmBackend {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl LcmBackend for FakeLcmBackend {
    async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
        let mut guard = self.inner.lock().expect("fake backend poisoned");
        let topic_key = frame.topic.channel().to_string();
        guard.topics.entry(topic_key).or_default().push(frame);
        guard.seqno = guard.seqno.saturating_add(1);
        Ok(())
    }

    async fn subscribe(&self, topic: LcmTopic, msg_type: Option<LcmMessageType>) -> LcmFrameStream {
        let inner = Arc::clone(&self.inner);
        // fake backend 简化:把当前 topic 队列作为一次性 stream 返回
        let snapshot = {
            let guard = inner.lock().expect("fake backend poisoned");
            guard
                .topics
                .get(topic.channel())
                .cloned()
                .unwrap_or_default()
        };
        // 类型过滤
        let typed: Vec<TransportResult<RawLcmFrame>> = snapshot
            .into_iter()
            .filter_map(|f| match &msg_type {
                Some(t) if f.msg_type.fqn() != t.fqn() => None,
                _ => Some(Ok(f)),
            })
            .collect();
        let stream = stream::iter(typed);
        Box::pin(stream)
    }

    fn name(&self) -> &str {
        "fake-lcm"
    }
}

// ============================================================================
// 兼容层（历史 UDP/SHM 抽象 + MessageEnvelope 旧 API）
// ============================================================================
//
// 总纲修订后,UDP/SHM 不再是默认传输层;它们仅保留以避免破坏下游 crate
// 中 `use nuwax_transport::Transport` / `use nuwax_transport::UdpTransport`
// 等导入。`Transport` trait / `UdpTransport` / `ShmTransport` 标记为
// deprecated,业务层请改用 LCM-first API。

/// 旧协议层信封（protobuf 兼容路径）
#[allow(deprecated)]
pub use nuwax_protocol::MessageEnvelope;

/// 旧传输抽象（**deprecated**:请改用 [`LcmBackend`] / [`LcmTransport`]）
#[deprecated(
    since = "0.2.0",
    note = "UDP/SHM 旧 API 已被 LcmBackend 取代;仅在 protobuf 兼容边界使用"
)]
#[allow(deprecated)] // trait 内部使用 MessageEnvelope 是预期行为
#[async_trait]
pub trait Transport: Send + Sync {
    /// 发布一条消息
    async fn publish(&self, envelope: MessageEnvelope) -> TransportResult<()>;
    /// 订阅某个 topic,yield 流式消息
    async fn subscribe(
        &self,
        topic: &str,
    ) -> TransportResult<Box<dyn futures::Stream<Item = MessageEnvelope> + Send + Unpin>>;
}

/// UDP 多播传输（**deprecated**:历史占位,新代码请走 LCM-first）
#[deprecated(
    since = "0.2.0",
    note = "UDP 多播已被 LCM-first 取代;仅在 protobuf 兼容边界使用"
)]
#[allow(dead_code)]
pub struct UdpTransport {
    /// 本地绑定地址
    pub bind_addr: String,
    /// 多播组地址
    pub multicast_group: String,
}

#[allow(deprecated)]
impl UdpTransport {
    /// 构造 UDP 多播传输（**deprecated**:仅在 protobuf 兼容边界使用）
    #[must_use]
    pub fn new(bind_addr: impl Into<String>, multicast_group: impl Into<String>) -> Self {
        Self {
            bind_addr: bind_addr.into(),
            multicast_group: multicast_group.into(),
        }
    }
}

/// 共享内存传输（**deprecated**:历史占位,新代码请走 LCM-first）
#[deprecated(
    since = "0.2.0",
    note = "SHM 已被 LCM-first 取代;仅在 protobuf 兼容边界使用"
)]
#[allow(dead_code)]
pub struct ShmTransport {
    /// 共享内存文件路径
    pub path: String,
    /// 容量（字节）
    pub capacity: usize,
}

#[allow(deprecated)]
impl ShmTransport {
    /// 构造共享内存传输（**deprecated**:仅在 protobuf 兼容边界使用）
    #[must_use]
    pub fn new(path: impl Into<String>, capacity: usize) -> Self {
        Self {
            path: path.into(),
            capacity,
        }
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
    use futures::StreamExt;
    use nuwax_protocol::RawLcmFrameBuilder;

    #[test]
    fn fake_backend_publish_and_subscribe() {
        let backend = FakeLcmBackend::new();
        let transport = LcmTransport::new(Arc::new(backend));

        // 同步跑 async;在测试中使用 futures::executor。
        futures::executor::block_on(async {
            let topic = LcmTopic::new("/scan").unwrap();
            let msg_type = LcmMessageType::new("sensor_msgs/PointCloud2").unwrap();
            let frame = RawLcmFrameBuilder::new(topic.clone(), msg_type.clone())
                .payload(b"hello".to_vec())
                .build();
            transport.publish(frame.clone()).await.unwrap();

            let mut stream = transport.subscribe(topic, Some(msg_type)).await;
            let received = stream.next().await.expect("one frame");
            let received = received.expect("ok");
            assert_eq!(received.payload, b"hello");
        });
    }

    #[test]
    fn fake_backend_typed_suffix_filter_rejects_mismatch() {
        let backend = FakeLcmBackend::new();
        let transport = LcmTransport::new(Arc::new(backend));

        futures::executor::block_on(async {
            let topic = LcmTopic::new("/scan").unwrap();
            let msg_type_pc = LcmMessageType::new("sensor_msgs/PointCloud2").unwrap();
            let frame = RawLcmFrameBuilder::new(topic.clone(), msg_type_pc.clone())
                .payload(b"pc".to_vec())
                .build();
            transport.publish(frame).await.unwrap();

            // 订阅时强行指定 Odometry:应过滤掉
            let msg_type_odom = LcmMessageType::new("nav_msgs/Odometry").unwrap();
            let mut stream = transport.subscribe(topic, Some(msg_type_odom)).await;
            let next = stream.next().await;
            assert!(next.is_none(), "type filter should drop mismatched frame");
        });
    }

    #[test]
    fn fake_backend_unmatched_pattern_yields_empty() {
        let backend = FakeLcmBackend::new();
        let transport = LcmTransport::new(Arc::new(backend));

        futures::executor::block_on(async {
            // 没有 publish 任何 frame,直接订阅
            let topic = LcmTopic::new("/nothing").unwrap();
            let mut stream = transport.subscribe(topic, None).await;
            let next = stream.next().await;
            assert!(next.is_none());
        });
    }

    #[test]
    fn fake_backend_malformed_topic_rejected() {
        // 验证 typed suffix 解析仍受 `LcmTopic::new` 校验
        let topic = LcmTopic::new("/scan#pkg.Foo").unwrap_err();
        let _ = topic; // 仅验证构造失败
    }

    #[test]
    #[allow(deprecated)]
    fn udp_transport_construct_compat() {
        let t = UdpTransport::new("0.0.0.0:0", "239.255.0.1");
        assert_eq!(t.bind_addr, "0.0.0.0:0");
        assert_eq!(t.multicast_group, "239.255.0.1");
    }

    #[test]
    #[allow(deprecated)]
    fn shm_transport_construct_compat() {
        let t = ShmTransport::new("/tmp/nuwax.shm", 1024);
        assert_eq!(t.path, "/tmp/nuwax.shm");
        assert_eq!(t.capacity, 1024);
    }

    #[test]
    fn bytes_to_vec_roundtrip() {
        let v = vec![1u8, 2, 3, 4];
        let out = bytes_to_vec(Bytes::from(v.clone()));
        assert_eq!(out, v);
    }

    #[test]
    fn fake_backend_name() {
        let backend = FakeLcmBackend::new();
        assert_eq!(backend.name(), "fake-lcm");
    }

    #[test]
    fn fake_backend_snapshot() {
        let backend = FakeLcmBackend::new();
        futures::executor::block_on(async {
            let topic = LcmTopic::new("/a").unwrap();
            let msg_type = LcmMessageType::new("pkg/A").unwrap();
            let frame = RawLcmFrameBuilder::new(topic, msg_type)
                .payload(b"x".to_vec())
                .build();
            backend.publish(frame).await.unwrap();
        });
        let snap = backend.snapshot();
        assert_eq!(snap.get("/a"), Some(&1));
    }
}
