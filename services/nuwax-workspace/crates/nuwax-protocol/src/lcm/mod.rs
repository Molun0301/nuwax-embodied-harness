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
//! +---------+--------+----------------+
//! |  magic  | seqno  |     payload    |
//! | 4B BE   | 4B BE  |     bytes      |
//! +---------+--------+----------------+
//! ```
//!
//! - **magic** 固定 0x4c433032 (4 字节大端 BE = `"LC02"` ASCII),与上游 LCM 1.x
//!   一致。**禁止**再以 little-endian 解析 —— 2026-07-09 B3 修正。
//! - **seqno** = 4 字节大端 BE channel sequence,生产端递增;消费端可忽略。
//! - **payload** = 字节流;长度由 wire buffer 自身边界决定,**不**写入
//!   `payload_len` 字段。
//!
//! ## 历史(2026-07-09 之前)
//!
//! 旧实现曾用 **28-byte envelope**:`magic(8) + fingerprint(8) + seqno(8) +
//! payload_len(4)`。该结构与 liblcm 1.x 真实 UDP multicast wire format 不兼容,
//! 已被 B3 audit 标记为 BLOCKING 并替换:
//! - `fingerprint` **不**出现在 wire 上 —— 类型识别由 LCM channel typed
//!   suffix(`/topic#pkg.Msg`)在订阅端承担(见 [`LcmTopic::as_subscription_pattern`])。
//! - `payload_len` 字段移除 —— 接收端用 `rbuf.data_size` 推算长度。
//! - fingerprint 数值仍由 [`LcmMessageType::fingerprint`] 给出(8 字节大端
//!   进程内表示),仅供进程内强类型契约,不写入 wire。
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
