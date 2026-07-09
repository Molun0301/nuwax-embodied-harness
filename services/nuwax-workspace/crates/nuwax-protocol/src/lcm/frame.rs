//! LCM raw frame —— wire-format 字节布局的强类型建模。
//!
//! ## Wire layout (LCM 1.x UDP multicast,2026-07-09 B3 audit 修正后)
//!
//! ```text
//!  0       1       2       3       4       5       6       7
//! +-------+-------+-------+-------+-------+-------+-------+-------+
//! |  'L'  |  'C'  |  '0'  |  '2'  |          seqno             |  8 bytes preamble
//! +-------+-------+-------+-------+-------+-------+-------+-------+
//! ```
//!
//! - **magic**(4 字节,大端网络序,`0x4c433032` = `"LC02"`)。注意上游
//!   liblcm 1.x 是把 magic 当作 ASCII 字符流写入 wire(`[L, C, 0, 2]`),
//!   在接收端被读作 `u32::from_be_bytes([L, C, 0, 2])` = `0x4c433032`。
//!   **禁止**再以 little-endian 解析。
//! - **seqno**(4 字节,u32,大端网络序)—— channel 内的递增序号。
//! - **payload 长度不写入 wire**:接收端用 `rbuf.data_size` 取得;`payload`
//!   紧跟 8 字节 preamble,长度由 wire buffer 自身边界决定。
//!
//! ## 历史
//!
//! 旧实现(2026-07-09 之前)曾用 **28 字节 envelope**:
//! `8 magic + 8 fingerprint + 8 seqno + 4 payload_len`,该结构与 liblcm 1.x
//! 真实 wire format 不兼容,已被 B3 audit 标记为 BLOCKING 并替换。`fingerprint`
//! 在 real wire 上根本不在 envelope 中(它由 channel-name typed suffix 表达,
//! 见 `LcmTopic::as_subscription_pattern` / `parse_subscription_pattern`),
//! 也不再有 `payload_len` 字段。

use serde::{Deserialize, Serialize};

use super::topic::LcmTopic;
use crate::ProtocolError;

/// LCM 1.x frame magic (big-endian u32 on wire = `"LC02"` = `0x4c433032`)
///
/// 大端写入 wire 后字节序列为 `[0x4c, 0x43, 0x30, 0x32]` = `"LC02"`(ASCII)。
pub const LCM_MAGIC: u32 = 0x4c43_3032;

/// magic 字段在 frame 中占 4 字节
pub const LCM_MAGIC_BYTES: usize = 4;
/// sequence number 字段 4 字节
pub const LCM_SEQNO_BYTES: usize = 4;

/// LCM frame header(8 字节 preamble,不含 payload)
///
/// 与上游 LCM 1.x UDP multicast wire format 一致;不含 fingerprint /
/// payload_len —— 这些字段都不在真实 wire 上。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LcmFrameHeader {
    /// magic(frame 第 0..4 字节的大端 u32)
    pub magic: u32,
    /// sequence number(4 字节,大端)
    pub seqno: u32,
}

impl LcmFrameHeader {
    /// header 总字节数(不含 payload)= 8 字节
    pub const HEADER_BYTES: usize = LCM_MAGIC_BYTES + LCM_SEQNO_BYTES;

    /// 编码为字节 buffer(不含 payload)
    pub fn encode(&self) -> [u8; Self::HEADER_BYTES] {
        let mut buf = [0u8; Self::HEADER_BYTES];
        buf[0..4].copy_from_slice(&self.magic.to_be_bytes());
        buf[4..8].copy_from_slice(&self.seqno.to_be_bytes());
        buf
    }

    /// 解析字节 buffer 为 header
    pub fn decode(buf: &[u8]) -> Result<Self, ProtocolError> {
        if buf.len() < Self::HEADER_BYTES {
            return Err(ProtocolError::Incomplete {
                need: Self::HEADER_BYTES,
                got: buf.len(),
            });
        }
        let magic = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if magic != LCM_MAGIC {
            return Err(ProtocolError::LcmDecode(format!(
                "bad magic: got 0x{magic:08x}, expected 0x{LCM_MAGIC:08x}",
            )));
        }
        let seqno = u32::from_be_bytes([buf[4], buf[5], buf[6], buf[7]]);
        Ok(Self { magic, seqno })
    }
}

/// LCM 原始 frame(topic + msg_type + 已编码 header + payload)
///
/// 8 字节 preamble(LCM magic BE + seqno BE)+ payload。
///
/// 注意:wire 上**没有** fingerprint 字段 —— 类型信息由 LCM channel
/// typed suffix(`/topic#pkg.Msg`)在订阅端承担(见
/// [`LcmTopic::as_subscription_pattern`]);`msg_type` 字段保留仅供
/// `RawLcmFrame` 在 Rust 进程内传递类型标签使用。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RawLcmFrame {
    /// LCM channel
    pub topic: LcmTopic,
    /// LCM message type(Rust 内部 type tag,不进入 wire)
    pub msg_type: crate::LcmMessageType,
    /// 已编码 header
    pub header: LcmFrameHeader,
    /// 消息 payload 字节
    pub payload: Vec<u8>,
}

impl RawLcmFrame {
    /// 暴露 LCM magic 常量
    pub const LCM_MAGIC: u32 = LCM_MAGIC;

    /// 编码为完整字节 buffer(header + payload)
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(LcmFrameHeader::HEADER_BYTES + self.payload.len());
        out.extend_from_slice(&self.header.encode());
        out.extend_from_slice(&self.payload);
        out
    }

    /// 从字节 buffer 解码 frame;topic / msg_type 由外部 `lookup` 提供
    /// (wire 上没有 fingerprint 字段,无法反查原始 FQN 字符串)。
    ///
    /// payload 长度直接由 `buf.len() - HEADER_BYTES` 推出,无需 wire 上的
    /// `payload_len` 字段。
    pub fn decode(
        buf: &[u8],
        topic: LcmTopic,
        msg_type: crate::LcmMessageType,
    ) -> Result<Self, ProtocolError> {
        let header = LcmFrameHeader::decode(buf)?;
        let payload_start = LcmFrameHeader::HEADER_BYTES;
        let payload_end = buf.len();
        if payload_end < payload_start {
            return Err(ProtocolError::Incomplete {
                need: payload_start,
                got: payload_end,
            });
        }
        Ok(Self {
            topic,
            msg_type,
            header,
            payload: buf[payload_start..payload_end].to_vec(),
        })
    }
}

/// `RawLcmFrame` 构造器(builder pattern)
#[derive(Debug, Clone)]
pub struct RawLcmFrameBuilder {
    topic: LcmTopic,
    msg_type: crate::LcmMessageType,
    payload: Vec<u8>,
    seqno: u32,
}

impl RawLcmFrameBuilder {
    /// 构造 builder
    #[must_use]
    pub fn new(topic: LcmTopic, msg_type: crate::LcmMessageType) -> Self {
        Self {
            topic,
            msg_type,
            payload: Vec::new(),
            seqno: 0,
        }
    }

    /// 设置 payload
    #[must_use]
    pub fn payload(mut self, payload: Vec<u8>) -> Self {
        self.payload = payload;
        self
    }

    /// 设置 sequence number
    #[must_use]
    pub fn seqno(mut self, seqno: u32) -> Self {
        self.seqno = seqno;
        self
    }

    /// 构造 frame
    #[must_use]
    pub fn build(self) -> RawLcmFrame {
        let header = LcmFrameHeader {
            magic: LCM_MAGIC,
            seqno: self.seqno,
        };
        RawLcmFrame {
            topic: self.topic,
            msg_type: self.msg_type,
            header,
            payload: self.payload,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LcmMessageType;

    #[test]
    fn header_encode_decode_roundtrip() {
        let header = LcmFrameHeader {
            magic: LCM_MAGIC,
            seqno: 42,
        };
        let buf = header.encode();
        assert_eq!(buf.len(), LcmFrameHeader::HEADER_BYTES);
        assert_eq!(buf.len(), 8, "preamble 必须是 8 字节");
        let parsed = LcmFrameHeader::decode(&buf).unwrap();
        assert_eq!(parsed, header);
    }

    #[test]
    fn header_magic_is_be_lc02_ascii() {
        let header = LcmFrameHeader {
            magic: LCM_MAGIC,
            seqno: 0,
        };
        let buf = header.encode();
        // magic 4 字节 = ASCII "LC02"
        assert_eq!(&buf[0..4], b"LC02");
    }

    #[test]
    fn header_decode_rejects_bad_magic() {
        // magic 写成非 `"LC02"` 的 4 字节 BE u32 —— 应被解码器拒绝。
        let mut buf = [0u8; LcmFrameHeader::HEADER_BYTES];
        // 任意非 LCM magic 的 BE u32,如 `0xdead_beef`
        buf[0..4].copy_from_slice(&0xdeadbeef_u32.to_be_bytes());
        let err = LcmFrameHeader::decode(&buf).unwrap_err();
        assert!(matches!(err, ProtocolError::LcmDecode(_)));
    }

    #[test]
    fn frame_encode_decode_roundtrip() {
        let topic = LcmTopic::new("/scan").unwrap();
        let msg_type = LcmMessageType::new("sensor_msgs/PointCloud2").unwrap();
        let frame = RawLcmFrameBuilder::new(topic.clone(), msg_type.clone())
            .payload(b"hello world".to_vec())
            .seqno(7)
            .build();
        let bytes = frame.encode();
        let parsed = RawLcmFrame::decode(&bytes, topic, msg_type).unwrap();
        assert_eq!(parsed.payload, b"hello world");
        assert_eq!(parsed.header.seqno, 7);
    }

    #[test]
    fn frame_total_size_is_preamble_plus_payload() {
        let topic = LcmTopic::new("/x").unwrap();
        let msg_type = LcmMessageType::new("pkg/Foo").unwrap();
        let payload = vec![0xABu8; 100];
        let frame = RawLcmFrameBuilder::new(topic, msg_type)
            .payload(payload.clone())
            .seqno(1)
            .build();
        let bytes = frame.encode();
        assert_eq!(bytes.len(), 8 + payload.len());
    }

    #[test]
    fn frame_decode_rejects_truncated() {
        let buf = vec![0u8; 4];
        let err = LcmFrameHeader::decode(&buf).unwrap_err();
        assert!(matches!(err, ProtocolError::Incomplete { .. }));
    }

    #[test]
    fn frame_decode_uses_buf_length_as_payload_size() {
        // 关键回归:不再依赖 wire 上的 payload_len 字段。
        // decode 时 payload 长度直接由 `buf.len() - 8` 推出 —— 任何
        // 跟在 preamble 后的字节都会被读作 payload。
        let topic = LcmTopic::new("/t").unwrap();
        let msg_type = LcmMessageType::new("pkg/Bar").unwrap();
        let payload = b"abcde".to_vec();
        let frame = RawLcmFrameBuilder::new(topic.clone(), msg_type.clone())
            .payload(payload.clone())
            .seqno(99)
            .build();
        let bytes = frame.encode();
        assert_eq!(bytes.len(), 8 + payload.len());

        // 在尾部追加额外字节 —— decode 应把"所有剩余"当作 payload,
        // 不再读 payload_len 字段。这意味着 receiver 在真实 liblcm
        // 路径上看到的 rbuf.data_size 就等于 payload 长度。
        let mut bloated = bytes.clone();
        bloated.extend_from_slice(b"TAIL");
        let parsed = RawLcmFrame::decode(&bloated, topic, msg_type).unwrap();
        // 收到的 payload = 原 payload + 追加的尾部 = "abcdeTAIL"
        let mut expected = payload.clone();
        expected.extend_from_slice(b"TAIL");
        assert_eq!(parsed.payload, expected);
    }
}