//! LCM typed message codec helper(协议层)
//!
//! ## Phase 2 (2026-07-07) 引入
//!
//! `docs/开发文档/建图导航探索重构/2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/00_最终执行方案.md §5 Phase 2`:
//!
//! - `build_lcm_message_bytes`:把 `(channel, msg_type, payload)` 包装成
//!   [`LcmMessageBytes`]。fingerprint 由 `nuwax-codec::hash::compute_real_lcm_fingerprint`
//!   提供,与上游 liblcm 1.x 等价。
//! - `parse_lcm_message_bytes`:从自定义 envelope 中反解出
//!   `(channel, fingerprint, payload)` —— **仅** 解析 `build_lcm_message_bytes`
//!   生成的 envelope,**不**用于解析上游 liblcm `lcm_recv_message` 的
//!   wire bytes(那是 Phase 3 `nuwax-lcm-sys` 的工作)。
//!
//! ## 与 `RawLcmFrame` 的关系
//!
//! - [`RawLcmFrame`] = 28-byte LC02 envelope + payload(fake backend 用);
//! - [`LcmMessageBytes`] = 字段强类型(供 real-lcm `lcm_publish` 调用)。
//!
//! 两者**不**可互转 —— `encode_for_real_lcm_publish` 显式返回 payload 切片,
//! `RawLcmFrame::encode` 拼装 envelope,以避免 double framing 误用。
//!
//! ## 不依赖 `nuwax-codec`
//!
//! `nuwax-protocol` 反向依赖 `nuwax-codec`(已在 Cargo.toml 声明);本模块
//! 只读取 `nuwax_codec::hash::compute_real_lcm_fingerprint` 与
//! `nuwax_codec::hash::lookup_seed`,不引入 codec 其它类型。

use crate::lcm::message_bytes::LcmMessageBytes;
use crate::{LcmMessageType, LcmTopic};
use nuwax_codec::hash::{fingerprint as compute_real_lcm_fingerprint, lookup_seed};

/// 错误类型
#[derive(Debug, thiserror::Error)]
pub enum TypedCodecError {
    /// LCM 编码失败
    #[error("lcm encode failed: {0}")]
    LcmEncode(String),
    /// LCM 解码失败
    #[error("lcm decode failed: {0}")]
    LcmDecode(String),
    /// 输入数据不完整
    #[error("incomplete input: need {need} bytes, got {got}")]
    Incomplete {
        /// 需要字节
        need: usize,
        /// 实际字节
        got: usize,
    },
}

/// 构造 [`LcmMessageBytes`],fingerprint 取自 STRUCT_SEEDS(上游 LCM 1.x 兼容)
pub fn build_lcm_message_bytes(
    channel: &str,
    msg_type: &str,
    payload: &[u8],
    sequence: u64,
) -> Result<LcmMessageBytes, TypedCodecError> {
    if channel.is_empty() || !channel.starts_with('/') {
        return Err(TypedCodecError::LcmEncode(format!(
            "topic 必须以 `/` 开头: {channel}"
        )));
    }
    if channel.contains('#') {
        return Err(TypedCodecError::LcmEncode(format!(
            "topic 不应含 typed suffix 分隔符 `#`: {channel}"
        )));
    }
    let msg_type = LcmMessageType::new(msg_type)
        .map_err(|e| TypedCodecError::LcmEncode(format!("invalid msg_type: {e}")))?;
    let fingerprint = compute_real_lcm_fingerprint(msg_type.fqn(), 0).ok_or_else(|| {
        TypedCodecError::LcmEncode(format!(
            "fingerprint 未在 STRUCT_SEEDS 注册(上游 lcm-gen 无对应 schema): {msg_type}"
        ))
    })?;
    let _topic = LcmTopic::new(channel)
        .map_err(|e| TypedCodecError::LcmEncode(format!("invalid channel: {e}")))?;
    Ok(LcmMessageBytes::new(
        channel,
        fingerprint,
        sequence,
        payload.to_vec(),
    ))
}

/// 从 codec 自定义 envelope 中反解
///
/// envelope 格式:
/// - 4-byte LE `u32` channel 长度
/// - channel bytes(UTF-8)
/// - 8-byte fingerprint(大端)
/// - payload bytes
///
/// 上游 liblcm `lcm_recv_message` 输出的 wire bytes 由 Phase 3 `nuwax-lcm-sys`
/// 单独处理,**不**走该函数。
pub fn parse_lcm_message_bytes(bytes: &[u8]) -> Result<(String, [u8; 8], Vec<u8>), TypedCodecError> {
    if bytes.len() < 4 {
        return Err(TypedCodecError::LcmDecode(format!(
            "envelope too short: need 4 bytes channel_len, got {}",
            bytes.len()
        )));
    }
    let channel_len = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) as usize;
    let channel_end = 4 + channel_len;
    if bytes.len() < channel_end {
        return Err(TypedCodecError::Incomplete {
            need: channel_end,
            got: bytes.len(),
        });
    }
    let channel = std::str::from_utf8(&bytes[4..channel_end])
        .map_err(|e| TypedCodecError::LcmDecode(format!("channel not utf-8: {e}")))?
        .to_string();
    let fp_end = channel_end + 8;
    if bytes.len() < fp_end {
        return Err(TypedCodecError::Incomplete {
            need: fp_end,
            got: bytes.len(),
        });
    }
    let mut fingerprint = [0u8; 8];
    fingerprint.copy_from_slice(&bytes[channel_end..fp_end]);
    let payload = bytes[fp_end..].to_vec();
    Ok((channel, fingerprint, payload))
}

/// 在 lookup_seed 注册检查上游 fingerprint 是否可用 —— Phase 2 fail-fast
/// gate。`msg_type` FQN 字符串即可。
#[must_use]
pub fn upstream_fingerprint_registered(msg_type: &str) -> bool {
    lookup_seed(msg_type).is_some()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_and_parse_roundtrip() {
        let msg = build_lcm_message_bytes("/odom", "nav_msgs/Odometry", b"hello", 7)
            .expect("ok");
        let envelope = encode_custom_envelope(&msg);
        let (channel, fp, payload) = parse_lcm_message_bytes(&envelope).expect("parse ok");
        assert_eq!(channel, "/odom");
        assert_eq!(fp, msg.fingerprint);
        assert_eq!(payload, b"hello");
    }

    #[test]
    fn build_rejects_invalid_channel() {
        let r = build_lcm_message_bytes("odom", "nav_msgs/Odometry", b"x", 0);
        assert!(matches!(r, Err(TypedCodecError::LcmEncode(_))));
    }

    #[test]
    fn build_rejects_unregistered_msg_type() {
        let r = build_lcm_message_bytes("/x", "totally_unknown/PkgType", b"x", 0);
        assert!(matches!(r, Err(TypedCodecError::LcmEncode(_))));
    }

    #[test]
    fn parse_rejects_short_buffer() {
        let r = parse_lcm_message_bytes(&[1, 2]);
        assert!(matches!(
            r,
            Err(TypedCodecError::LcmDecode(_)) | Err(TypedCodecError::Incomplete { .. })
        ));
    }

    #[test]
    fn upstream_fingerprint_registered_works() {
        assert!(upstream_fingerprint_registered("geometry_msgs/Twist"));
        assert!(!upstream_fingerprint_registered("not/Registered"));
    }

    fn encode_custom_envelope(msg: &LcmMessageBytes) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&(msg.channel.len() as u32).to_le_bytes());
        out.extend_from_slice(msg.channel.as_bytes());
        out.extend_from_slice(&msg.fingerprint);
        out.extend_from_slice(&msg.payload);
        out
    }
}