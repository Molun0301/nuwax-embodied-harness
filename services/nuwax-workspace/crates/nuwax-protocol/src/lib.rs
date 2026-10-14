//! nuwax 消息协议层
//!
//! ## LCM-first 设计口径（总纲 §10 / 00-执行规划总纲.md 修订）
//!
//! - 本 crate 把 **LCM** 作为默认消息协议层；wire format 与 `lcm_msgs`
//!   (Python/C++/Rust) 保持 byte-for-byte 兼容。
//! - `protobuf / prost / tonic / gRPC` **不**作为默认协议层；它们仅在
//!   "确有上游 protobuf 兼容需求"的边界被显式启用。
//! - LCM 消息以 **8 字节 preamble**(4 字节 magic `0x4c433032` = `"LC02"` 大端
//!   网络序 + 4 字节 seqno 大端)+ payload 组成。Fingerprint **不**写入
//!   wire —— 由 LCM channel typed suffix(`/topic#pkg.Msg`)在订阅端承担
//!   类型识别(见 [`LcmTopic::as_subscription_pattern`])。Fingerprint 数值
//!   沿用上游 `lcm-gen` 的 `cityhash64 / MurmurHash64A` 协议(以 8 字节
//!   大端网络序在进程内传递,如 [`LcmMessageType::fingerprint`])。
//!
//! ## P0 边界（本文件声明的 foundation milestone）
//!
//! - **完成**：`LcmTopic` / `LcmMessageType` / `RawLcmFrame` / `LcmFrameHeader` /
//!   LCM framer/`lcm_hash` 常量构造与基础契约。
//! - **不做**：native bridge、rosbag、aarch64 全链路验证、完整 Python/C++/Rust
//!   三方真实对账（属于 P1）；`real-lcm` backend 走 `nuwax-transport` 的 feature gate。
//!
//! # 用法示例
//!
//! ```no_run
//! use nuwax_protocol::{LcmTopic, LcmMessageType, RawLcmFrame, RawLcmFrameBuilder};
//!
//! let topic = LcmTopic::new("/scan").expect("valid topic");
//! let msg_type = LcmMessageType::new("sensor_msgs/PointCloud2").expect("valid type");
//! let frame: RawLcmFrame = RawLcmFrameBuilder::new(topic, msg_type)
//!     .payload(b"hello".to_vec())
//!     .build();
//! assert_eq!(frame.header.magic, RawLcmFrame::LCM_MAGIC);
//! // wire = 8 字节 preamble + payload (no fingerprint / payload_len on wire)
//! assert_eq!(frame.encode().len(), 8 + frame.payload.len());
//! ```

#![deny(unsafe_code)]
#![warn(missing_docs)]

use thiserror::Error;

pub mod lcm;

/// 协议错误（保留为顶级 alias：避免破坏依赖 `nuwax_protocol::ProtocolError`
/// 的下游 crate re-export）
#[derive(Debug, Error)]
pub enum ProtocolError {
    /// LCM 编码失败
    #[error("lcm encode failed: {0}")]
    LcmEncode(String),
    /// LCM 解码失败
    #[error("lcm decode failed: {0}")]
    LcmDecode(String),
    /// codec 错误
    #[error("codec error: {0}")]
    Codec(#[from] nuwax_codec::CodecError),
    /// 输入数据不完整
    #[error("incomplete input: need {need} bytes, got {got}")]
    Incomplete {
        /// 需要字节
        need: usize,
        /// 实际字节
        got: usize,
    },
}

// ============================================================================
// LCM-first 公共 API
// ============================================================================
//
// 这些是默认协议层类型。`LcmTopic` / `LcmMessageType` / `RawLcmFrame` 是
// LCM wire-format 的强类型建模:topic 带 typed suffix（`/topic#pkg.Msg`）
// 由 `LcmMessageType::fingerprint` 提供 8 字节 `lcm_hash`，与上游
// `lcm-gen` 完全兼容。
pub use lcm::{
    build_lcm_message_bytes, parse_lcm_message_bytes, upstream_fingerprint_registered,
    LcmFrameHeader, LcmMessageBytes, LcmMessageType, LcmTopic, LcmTopicParseError, RawLcmFrame,
    RawLcmFrameBuilder, TypedCodecError, LCM_HASH_SIZE, LCM_MAGIC, LCM_PROTOCOL_MAJOR,
    LCM_PROTOCOL_MINOR, LCM_TOPIC_TYPED_SUFFIX_SEP,
};

/// 消息主题、类型、帧与错误的常用导出。
pub mod prelude {
    pub use crate::{
        LcmFrameHeader, LcmMessageType, LcmTopic, ProtocolError, RawLcmFrame, LCM_MAGIC,
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RawLcmFrameBuilder;

    #[test]
    fn lcm_magic_is_lc02() {
        // LCM 1.x magic = ASCII "LC02" = 0x4c433032(big-endian on wire,2026-07-09 B3 修正)
        assert_eq!(RawLcmFrame::LCM_MAGIC, u32::from_be_bytes(*b"LC02"));
        assert_eq!(LCM_MAGIC, RawLcmFrame::LCM_MAGIC);
    }

    #[test]
    fn lcm_frame_builder_smoke() {
        let topic = LcmTopic::new("/scan").unwrap();
        let msg_type = LcmMessageType::new("sensor_msgs/PointCloud2").unwrap();
        let frame = RawLcmFrameBuilder::new(topic, msg_type)
            .payload(b"hello".to_vec())
            .build();
        assert_eq!(frame.payload.as_slice(), b"hello");
        assert_eq!(frame.header.magic, LCM_MAGIC);
        // 8 字节 preamble(LC02 magic BE + seqno BE)+ payload —— B3 audit 修正后
        assert_eq!(frame.encode().len(), 8 + frame.payload.len());
    }
}

/// LCM wire-format 协议版本（常量对象，与 `RawLcmFrame::LCM_MAGIC` 配套用于
/// handshake 校验）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LCMProtocolVersion {
    /// 主版本号
    pub major: u16,
    /// 次版本号
    pub minor: u16,
}

impl LCMProtocolVersion {
    /// 当前协议版本（与上游 LCM 1.x 一致）
    pub const CURRENT: Self = Self {
        major: LCM_PROTOCOL_MAJOR,
        minor: LCM_PROTOCOL_MINOR,
    };
}
