//! LCM-first 协议类型（默认协议层）
//!
//! 该模块把 LCM 1.x 的 wire-format 建模为强类型,允许下游 crate 直接消费
//! topic / message-type / raw frame 三个核心概念。
//!
//! 注意:本模块是 foundation milestone — 它 **不** 引入外部 `lcm_msgs`
//! crate 或 `liblcm` FFI binding。`real-lcm` backend 在 `nuwax-transport`
//! feature gate 后实现;此处仅固化 wire-format **字节布局**。
//!
//! ## Wire format（参考 [`RawLcmFrame`]）
//!
//! ```text
//! +---------+--------------------+--------+--------+----------------+
//! | magic   | message fingerprint| seqno  | payload| payload        |
//! | 8B "LC02"|  8B  lcm_hash      | 8B LE  | length | bytes          |
//! +---------+--------------------+--------+--------+----------------+
//! ```
//!
//! - **magic** 固定 0x4c433032 (8 字节，"LC02" + padding),与上游 LCM 1.x 一致。
//! - **message fingerprint** = 8 字节 `lcm_hash`,由 LCM message type
//!   字符串(`pkg::Type`)经 `lcm-gen` MurmurHash / cityhash64 计算;
//!   Rust 侧通过 [`LcmMessageType::fingerprint`] 提供常量构造与解析。
//! - **seqno** = 8 字节 little-endian channel sequence,生产端递增;消费端可忽略。
//! - **payload length** = 4 字节 little-endian。
//! - **payload** = 字节流。
//!
//! ## Topic typed suffix
//!
//! LCM 1.x 的 `lcm.subscribe("regex")` 接受 typed suffix 形式
//! `/topic#pkg.Msg`。本 crate 在 [`LcmTopic::new_with_typed_suffix`] 提供
//! 显式解析能力,在 [`LcmTopic::as_subscription_pattern`] 中输出
//! 标准 `regex` 模式以匹配上游 LCM。

pub mod fingerprint;
pub mod frame;
pub mod message_bytes;
pub mod message_type;
pub mod topic;
pub mod typed_codec;

pub use fingerprint::{compute_lcm_fingerprint, LCM_HASH_SIZE};
pub use frame::{LcmFrameHeader, RawLcmFrame, RawLcmFrameBuilder, LCM_MAGIC};
pub use message_bytes::LcmMessageBytes;
pub use message_type::{LcmMessageType, LcmMessageTypeError};
pub use topic::{LcmTopic, LcmTopicParseError, LCM_TOPIC_TYPED_SUFFIX_SEP};
pub use typed_codec::{
    build_lcm_message_bytes, parse_lcm_message_bytes, upstream_fingerprint_registered,
    TypedCodecError,
};

/// LCM 1.x 主版本号（写入 frame header）
pub const LCM_PROTOCOL_MAJOR: u16 = 1;
/// LCM 1.x 次版本号（写入 frame header）
pub const LCM_PROTOCOL_MINOR: u16 = 4;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn protocol_version_constants() {
        assert_eq!(LCM_PROTOCOL_MAJOR, 1);
        assert_eq!(LCM_PROTOCOL_MINOR, 4);
    }

    #[test]
    fn typed_suffix_separator_is_hash() {
        assert_eq!(LCM_TOPIC_TYPED_SUFFIX_SEP, '#');
    }
}
