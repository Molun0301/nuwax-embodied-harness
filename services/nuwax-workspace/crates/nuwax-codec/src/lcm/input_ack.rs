//! 原生扫描处理回执，布局以仓库中的 LioInputAck.lcm 为准。

use crate::{CodecError, CodecResult};

/// 由仓库消息定义生成的通信指纹。
pub const FINGERPRINT: [u8; 8] = 0x76c9_3e45_543d_ab19_u64.to_be_bytes();

/// 原生处理完成或拒绝的同源扫描身份。
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LioInputAck {
    /// 当前私有代际内从1递增的扫描编号。
    pub sequence: i32,
    /// 扫描源起始时间。
    pub scan_begin_timestamp_ns: u64,
    /// 扫描最大逐点时间对应的末端。
    pub scan_end_timestamp_ns: u64,
    /// 已完成原生处理迭代，不代表必定产生位姿。
    pub success: bool,
    /// 处理说明或拒绝原因。
    pub message: String,
}

/// 解码不含顶层指纹的消息体并校验身份。
pub fn decode_body(bytes: &[u8]) -> CodecResult<LioInputAck> {
    let invalid = |reason: &str| CodecError::LcmDecode(format!("LioInputAck: {reason}"));
    if bytes.len() < 30 {
        return Err(invalid("回执被截断"));
    }
    let i32_at = |offset| i32::from_be_bytes(bytes[offset..offset + 4].try_into().unwrap());
    let i64_at = |offset| i64::from_be_bytes(bytes[offset..offset + 8].try_into().unwrap());
    let begin = i64_at(8);
    let end = i64_at(16);
    let length = i32_at(25);
    if i32_at(0) != 1
        || i32_at(4) <= 0
        || begin < 0
        || end < begin
        || bytes[24] > 1
        || !(1..=1025).contains(&length)
        || bytes.len() != 29 + length as usize
        || bytes.last() != Some(&0)
    {
        return Err(invalid("版本、身份、扫描窗口或消息长度不合法"));
    }
    let message = std::str::from_utf8(&bytes[29..bytes.len() - 1])
        .map_err(|_| invalid("消息不是UTF-8文本"))?
        .to_owned();
    Ok(LioInputAck {
        sequence: i32_at(4),
        scan_begin_timestamp_ns: begin as u64,
        scan_end_timestamp_ns: end as u64,
        success: bytes[24] == 1,
        message,
    })
}

/// 编码消息体；与生成的C++/Python绑定具有相同布局。
pub fn encode_body(ack: &LioInputAck) -> CodecResult<Vec<u8>> {
    if ack.message.len() > 1024 {
        return Err(CodecError::LcmEncode("LioInputAck消息超过1024字节".into()));
    }
    let mut out = Vec::with_capacity(30 + ack.message.len());
    out.extend_from_slice(&1_i32.to_be_bytes());
    out.extend_from_slice(&ack.sequence.to_be_bytes());
    out.extend_from_slice(&(ack.scan_begin_timestamp_ns as i64).to_be_bytes());
    out.extend_from_slice(&(ack.scan_end_timestamp_ns as i64).to_be_bytes());
    out.push(u8::from(ack.success));
    out.extend_from_slice(&((ack.message.len() + 1) as i32).to_be_bytes());
    out.extend_from_slice(ack.message.as_bytes());
    out.push(0);
    decode_body(&out)?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn receipt_rejects_truncation_bad_identity_and_window() {
        let ack = LioInputAck {
            sequence: 7,
            scan_begin_timestamp_ns: 10,
            scan_end_timestamp_ns: 20,
            success: true,
            message: "processed".into(),
        };
        let encoded = encode_body(&ack).unwrap();
        assert_eq!(decode_body(&encoded).unwrap(), ack);
        for length in 0..encoded.len() {
            assert!(decode_body(&encoded[..length]).is_err());
        }
        let mut bad = ack.clone();
        bad.sequence = 0;
        assert!(encode_body(&bad).is_err());
        bad = ack;
        bad.scan_end_timestamp_ns = 9;
        assert!(encode_body(&bad).is_err());
    }
}
