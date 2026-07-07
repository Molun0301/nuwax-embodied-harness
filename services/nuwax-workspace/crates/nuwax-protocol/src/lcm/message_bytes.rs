//! `LcmMessageBytes`:real-LCM publish payload 强类型封装。
//!
//! ## Phase 2 (2026-07-07) 引入
//!
//! `docs/开发文档/建图导航探索重构/2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/00_最终执行方案.md §5 Phase 2`:
//!
//! LCM 1.x `lcm_publish(channel, data, datalen)` 需要的 `data` 是 **结构化消息
//! payload bytes**,**不**是内部 [`RawLcmFrame`] 那种 "LC02 + fingerprint + seqno +
//! payload_len + payload" envelope。
//!
//! [`LcmMessageBytes`] 把 "channel / fingerprint / sequence / payload" 四件
//! 套打包成单一类型,作为 Phase 3 `real-lcm` backend 的 wire-format
//! 合约:
//!
//! - `channel` = LCM topic(由 `lcm_publish` 第一参数消费);
//! - `fingerprint` = 8 字节大端(用于 channel 调度时校验类型);
//! - `sequence` = 8 字节递增(`lcm_publish` 不使用,仅 metadata);
//! - `payload` = 由 `lcm-gen` 生成的 encoder 写入的真实消息体。
//!
//! ## 与 RawLcmFrame 的区别
//!
//! - [`RawLcmFrame`]:包含 28-byte LC02 envelope,适合 fake backend 内部测试与
//!   raw log 回放;
//! - [`LcmMessageBytes`]:只持有消息体 + 元信息,**不**预先拼装 envelope。
//!   真实 `lcm_publish` 收到 LcmMessageBytes 后由 liblcm 自行加 envelope。
//!
//! ## 入库 / 导出
//!
//! `pub use` 在 crate root:`use nuwax_protocol::LcmMessageBytes;`

use serde::{Deserialize, Serialize};

/// real-LCM publish payload 字节封装
///
/// `LcmMessageBytes` 不直接对外暴露 `&[u8]` 视图(以避免误与
/// `RawLcmFrame::encode()` 输出混淆),所有对外消费必须经过
/// [`LcmMessageBytes::encode_for_real_lcm_publish`] 或 [`LcmMessageBytes::as_slice`]
/// 显式取走 payload 部分。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LcmMessageBytes {
    /// LCM channel (如 `/odom`)
    pub channel: String,
    /// 8 字节 message fingerprint(大端网络序)
    pub fingerprint: [u8; 8],
    /// 8 字节递增 sequence(写 liblcm metadata;`lcm_publish` 不消费)
    pub sequence: u64,
    /// payload 字节流(`lcm-gen` encoder 写入的真实消息体)
    pub payload: Vec<u8>,
}

impl LcmMessageBytes {
    /// 构造 new [`LcmMessageBytes`]
    ///
    /// 不做额外校验 —— `channel` / `fingerprint` 由调用方在 publish 路径
    /// 上游保证有效。
    #[must_use]
    pub fn new(
        channel: impl Into<String>,
        fingerprint: [u8; 8],
        sequence: u64,
        payload: Vec<u8>,
    ) -> Self {
        Self {
            channel: channel.into(),
            fingerprint,
            sequence,
            payload,
        }
    }

    /// 编码供 `lcm_publish(channel, data, datalen)` 使用的 slice
    ///
    /// 注意:**仅返回 `payload`**。`lcm_publish` 不会消费 channel / fingerprint /
    /// sequence —— channel 是它的第一个参数,fingerprint / sequence 由 liblcm
    /// 内部写入 envelope。该函数返回的 slice 是"消息体"部分,可以直接喂给
    /// 上游 C `lcm_publish(...)`。
    #[must_use]
    pub fn encode_for_real_lcm_publish(&self) -> &[u8] {
        &self.payload
    }

    /// 等价于 [`LcmMessageBytes::encode_for_real_lcm_publish`],语义别名
    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        &self.payload
    }

    /// 长度(字节)
    #[must_use]
    pub fn len(&self) -> usize {
        self.payload.len()
    }

    /// 是否为空 payload
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.payload.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_payload_view() {
        let bytes = LcmMessageBytes::new(
            "/odom",
            [0xbb, 0xca, 0x3b, 0x0d, 0x2c, 0x22, 0xf3, 0x84],
            1,
            vec![1, 2, 3, 4],
        );
        assert_eq!(bytes.channel, "/odom");
        assert_eq!(bytes.fingerprint[0], 0xbb);
        assert_eq!(bytes.sequence, 1);
        assert_eq!(bytes.encode_for_real_lcm_publish(), &[1, 2, 3, 4]);
        assert_eq!(bytes.as_slice(), &[1, 2, 3, 4]);
        assert_eq!(bytes.len(), 4);
        assert!(!bytes.is_empty());
    }

    #[test]
    fn empty_payload_is_empty() {
        let bytes = LcmMessageBytes::new("/x", [0u8; 8], 0, vec![]);
        assert!(bytes.is_empty());
        assert_eq!(bytes.encode_for_real_lcm_publish(), &[] as &[u8]);
    }

    #[test]
    fn serde_roundtrip() {
        let bytes = LcmMessageBytes::new(
            "/scan",
            [1, 2, 3, 4, 5, 6, 7, 8],
            42,
            b"point cloud bytes".to_vec(),
        );
        let json = serde_json::to_string(&bytes).unwrap();
        let back: LcmMessageBytes = serde_json::from_str(&json).unwrap();
        assert_eq!(back, bytes);
    }
}