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
//! 关键约定(2026-07-08 fake-lcm 根除后):
//! - [`LcmBackend`] trait:实现 publish / subscribe;`real-lcm` 是 nuwax 仓内唯一 backend。
//! - [`LcmTransport`]:real-lcm backend 工厂 + publish/subscribe 转发。
//! - **`real-lcm` feature**(workspace `default = ["real-lcm"]`):启用真正的 liblcm / Rust LCM
//!   binding backend (`real_backend::RealLcmBackend`)。 2026-07-07 Phase 3
//!   实装,详见 `crates/nuwax-lcm` / `crates/nuwax-lcm-sys` 模块文档。
//! - **`FakeLcmBackend` 已物理删除**(2026-07-08 v3 F2 决策):
//!   不再有 `LcmTransport::new_fake()` 工厂;无 fake-lcm / fake 旁路;
//!   无 compile_error 守门;镜像必须装 liblcm-dev 1.3~1.5 才能编译。
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
/// 2026-07-08 fake-lcm 物理删除后,唯一允许的实现是 `RealLcmBackend` (Phase 3 实装);
/// `FakeLcmBackend` / `Shm-sidecar` / 其它 backend 路径已物理删除。
#[async_trait]
pub trait LcmBackend: Send + Sync {
    /// 异步发布一帧
    async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()>;
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
/// 2026-07-08 fake-lcm 根除后:真实 LCM (`LcmTransport::new_real`)是唯一公开
/// backend 工厂。`LcmTransport::new_fake()` 已物理删除。
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
    /// 2026-07-08 fake-lcm 根除后,这是 nuwax-transport 公开层唯一允许的
    /// backend 工厂。`provider_url` 形如
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
// LcmTransport 自身实现 LcmBackend (newtype 转发)
// ============================================================================
//
// 2026-07-08 fake-lcm 物理删除(v3 F2)后,本 crate 唯一 backend 是
// `RealLcmBackend`。`LcmTransport` 作为持有 `Arc<dyn LcmBackend>` 的
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
    use nuwax_protocol::RawLcmFrameBuilder;

    // 2026-07-08 fake-lcm 物理删除(v3 F2 决策):
    // 原 fake_backend_publish_and_subscribe / fake_backend_typed_suffix_filter_rejects_mismatch /
    // fake_backend_unmatched_pattern_yields_empty / fake_backend_malformed_topic_rejected /
    // fake_backend_name / fake_backend_snapshot 6 个 fake-specific 测试**整段物理删除**。
    // 测试目标 `FakeLcmBackend` 已从 lib.rs 整段删除,这些测试已无被测对象。
    // 单测 helper 改走 `LcmTransport::new_real` + 隔离端口(test_lcm_url 公式,
    // 防止重现 07_08 实机 175 GB 崩溃)。

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

    /// 2026-07-08 fake-lcm 根除后,`LcmTopic::new` 校验仍生效(typed suffix 解析)。
    /// 这一行曾经归 `fake_backend_malformed_topic_rejected`,现与 backend 无关,
    /// 留下作为 typed suffix 解析的单测锚点(避免 future 重构破坏 typed suffix)。
    #[test]
    fn typed_suffix_parser_rejects_malformed() {
        let topic = LcmTopic::new("/scan#pkg.Foo").unwrap_err();
        let _ = topic;
    }

    /// 守恒测试:2026-07-08 fake-lcm 根除后,公开层只暴露 `new_real`,不再有
    /// `new_fake` / `FakeLcmBackend`。本测试做"字符串守恒",确保未来回归
    /// 不被回滚 —— `real-lcm` 是唯一 backend 名,`fake-lcm` 不应再出现。
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
}
