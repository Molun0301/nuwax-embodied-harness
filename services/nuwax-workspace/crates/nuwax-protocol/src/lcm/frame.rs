//! LCM raw frame —— wire-format 字节布局的强类型建模。
//!
//! ## Wire layout
//!
//! ```text
//!  0       1       2       3       4
//! +-------+-------+-------+-------+-------+-------+-------+-------+
//! |  'L'  |  'C'  |  '0'  |  '2'  |  0    |  0    |  0    |  0    |  8 bytes  magic
//! +-------+-------+-------+-------+-------+-------+-------+-------+
//! |        message fingerprint (8 bytes, big-endian lcm_hash)        |
//! +-------+-------+-------+-------+-------+-------+-------+-------+
//! |        sequence number          (8 bytes, little-endian)         |
//! +-------+-------+-------+-------+-------+-------+-------+-------+
//! |        payload length           (4 bytes, little-endian)         |
//! +-------+-------+-------+-------+
//! |        payload bytes            (variable, no padding)            |
//! +-------+-------+-------+-------+
//! ```
//!
//! 8 字节 magic 高位 4 字节固定为 `"LC02"`;后 4 字节为零（`lcm-gen` 兼容）。
//! 实际 wire 上 magic 以 **little-endian u32** 解析 → `0x4c433032`。

use serde::{Deserialize, Serialize};

use super::message_type::LcmMessageType;
use super::topic::LcmTopic;
use crate::ProtocolError;

/// LCM 1.x frame magic（little-endian u32 on wire = `"LC02"` = `0x3230_434c`）
pub const LCM_MAGIC: u32 = 0x3230_434c;

/// magic 字段在 frame 中占 8 字节;前 4 字节为 `LCM_MAGIC` (LE on wire),
/// 后 4 字节为 0。
pub const LCM_MAGIC_BYTES: usize = 8;
/// fingerprint 字段 8 字节
pub const LCM_FINGERPRINT_BYTES: usize = 8;
/// sequence number 字段 8 字节
pub const LCM_SEQNO_BYTES: usize = 8;
/// payload length 字段 4 字节
pub const LCM_PAYLOAD_LEN_BYTES: usize = 4;

/// LCM frame header（不含 payload）
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LcmFrameHeader {
    /// magic（frame 第 0..4 字节的 little-endian u32）
    pub magic: u32,
    /// message fingerprint（8 字节,大端）
    pub fingerprint: [u8; 8],
    /// sequence number（little-endian u64）
    pub seqno: u64,
    /// payload length（little-endian u32）
    pub payload_len: u32,
    /// 协议主版本
    pub protocol_major: u16,
    /// 协议次版本
    pub protocol_minor: u16,
}

impl LcmFrameHeader {
    /// header 总字节数（不含 payload）
    pub const HEADER_BYTES: usize =
        LCM_MAGIC_BYTES + LCM_FINGERPRINT_BYTES + LCM_SEQNO_BYTES + LCM_PAYLOAD_LEN_BYTES;

    /// 编码为字节 buffer（不含 payload）
    pub fn encode(&self) -> [u8; Self::HEADER_BYTES] {
        let mut buf = [0u8; Self::HEADER_BYTES];
        buf[0..4].copy_from_slice(&self.magic.to_le_bytes());
        // magic 4..8 字节保留为 0
        buf[8..16].copy_from_slice(&self.fingerprint);
        buf[16..24].copy_from_slice(&self.seqno.to_le_bytes());
        buf[24..28].copy_from_slice(&self.payload_len.to_le_bytes());
        // protocol_major/minor 不进入 wire,仅 metadata
        let _ = self.protocol_major;
        let _ = self.protocol_minor;
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
        let mut magic_bytes = [0u8; 4];
        magic_bytes.copy_from_slice(&buf[0..4]);
        let magic = u32::from_le_bytes(magic_bytes);
        if magic != LCM_MAGIC {
            return Err(ProtocolError::LcmDecode(format!(
                "bad magic: got 0x{magic:08x}, expected 0x{LCM_MAGIC:08x}",
            )));
        }
        let mut fingerprint = [0u8; 8];
        fingerprint.copy_from_slice(&buf[8..16]);
        let mut seqno_bytes = [0u8; 8];
        seqno_bytes.copy_from_slice(&buf[16..24]);
        let seqno = u64::from_le_bytes(seqno_bytes);
        let mut len_bytes = [0u8; 4];
        len_bytes.copy_from_slice(&buf[24..28]);
        let payload_len = u32::from_le_bytes(len_bytes);
        Ok(Self {
            magic,
            fingerprint,
            seqno,
            payload_len,
            protocol_major: super::LCM_PROTOCOL_MAJOR,
            protocol_minor: super::LCM_PROTOCOL_MINOR,
        })
    }
}

/// LCM 原始 frame（topic + msg_type + payload + 已编码字节）
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RawLcmFrame {
    /// LCM channel
    pub topic: LcmTopic,
    /// LCM message type
    pub msg_type: LcmMessageType,
    /// 已编码 header
    pub header: LcmFrameHeader,
    /// 消息 payload 字节
    pub payload: Vec<u8>,
}

impl RawLcmFrame {
    /// 暴露 LCM magic 常量
    pub const LCM_MAGIC: u32 = LCM_MAGIC;

    /// 编码为完整字节 buffer（header + payload）
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(LcmFrameHeader::HEADER_BYTES + self.payload.len());
        out.extend_from_slice(&self.header.encode());
        out.extend_from_slice(&self.payload);
        out
    }

    /// 从字节 buffer 解码 frame;topic / msg_type 由外部 `lookup` 提供
    /// (因为 fingerprint 不可逆反查到原始 FQN 字符串)
    pub fn decode(
        buf: &[u8],
        topic: LcmTopic,
        msg_type: LcmMessageType,
    ) -> Result<Self, ProtocolError> {
        let header = LcmFrameHeader::decode(buf)?;
        let payload_start = LcmFrameHeader::HEADER_BYTES;
        let payload_end = payload_start
            .checked_add(header.payload_len as usize)
            .ok_or_else(|| ProtocolError::LcmDecode("payload_len overflow".into()))?;
        if buf.len() < payload_end {
            return Err(ProtocolError::Incomplete {
                need: payload_end,
                got: buf.len(),
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

/// `RawLcmFrame` 构造器（builder pattern）
#[derive(Debug, Clone)]
pub struct RawLcmFrameBuilder {
    topic: LcmTopic,
    msg_type: LcmMessageType,
    payload: Vec<u8>,
    seqno: u64,
}

impl RawLcmFrameBuilder {
    /// 构造 builder
    #[must_use]
    pub fn new(topic: LcmTopic, msg_type: LcmMessageType) -> Self {
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
    pub fn seqno(mut self, seqno: u64) -> Self {
        self.seqno = seqno;
        self
    }

    /// 构造 frame
    #[must_use]
    pub fn build(self) -> RawLcmFrame {
        let payload_len =
            u32::try_from(self.payload.len()).expect("LCM payload 长度限制:不超过 4 GiB");
        let header = LcmFrameHeader {
            magic: LCM_MAGIC,
            fingerprint: self.msg_type.fingerprint(),
            seqno: self.seqno,
            payload_len,
            protocol_major: super::LCM_PROTOCOL_MAJOR,
            protocol_minor: super::LCM_PROTOCOL_MINOR,
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

    #[test]
    fn header_encode_decode_roundtrip() {
        let header = LcmFrameHeader {
            magic: LCM_MAGIC,
            fingerprint: [1, 2, 3, 4, 5, 6, 7, 8],
            seqno: 42,
            payload_len: 5,
            protocol_major: 1,
            protocol_minor: 4,
        };
        let buf = header.encode();
        assert_eq!(buf.len(), LcmFrameHeader::HEADER_BYTES);
        let parsed = LcmFrameHeader::decode(&buf).unwrap();
        assert_eq!(parsed, header);
    }

    #[test]
    fn header_decode_rejects_bad_magic() {
        let mut buf = [0u8; LcmFrameHeader::HEADER_BYTES];
        buf[0..4].copy_from_slice(&0xdeadbeef_u32.to_le_bytes());
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
    fn frame_decode_rejects_truncated() {
        let _topic = LcmTopic::new("/x").unwrap();
        let _msg_type = LcmMessageType::new("pkg/Foo").unwrap();
        let buf = vec![0u8; 4];
        let err = LcmFrameHeader::decode(&buf).unwrap_err();
        assert!(matches!(err, ProtocolError::Incomplete { .. }));
    }
}
