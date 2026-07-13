//! nuwax 序列化编解码基础
//!
//! ## LCM-first / protobuf-compat 双路径
//!
//! - 默认路径:LCM 消息编码(`lz4_flex` 压缩 + LCM frame 头 + 字节布局)
//!   由 [`nuwax-protocol`](https://docs.rs/nuwax-protocol) 承载;本 crate 仅
//!   在 [`lcm`] 模块提供 LCM golden fixture helper。
//! - 兼容路径:`protobuf` 仅在"确有上游 protobuf 兼容需求"的边界被启用。
//!   见 [`protobuf`] 模块。
//!
//! ## P0 foundation milestone 边界
//!
//! - `lcm` 模块:golden fixture 编码器 + lz4 压缩辅助。
//! - `protobuf` 模块:frame 长度前缀 / 解码辅助,标记为 compat-only。
//! - `jpeg` 模块:仅 lz4 + turbojpeg re-export。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use bytes::{Bytes, BytesMut};
use thiserror::Error;

pub mod hash;
pub mod lcm;
pub mod protobuf;

pub use hash::{
    fingerprint as compute_real_lcm_fingerprint, lcm_hash_64, lookup_seed, murmur_hash64a,
    STRUCT_SEEDS,
};

/// 编解码错误
#[derive(Debug, Error)]
pub enum CodecError {
    /// LCM 编码失败
    #[error("lcm encode failed: {0}")]
    LcmEncode(String),
    /// LCM 解码失败
    #[error("lcm decode failed: {0}")]
    LcmDecode(String),
    /// lz4 解压失败
    #[error("lz4 decompress failed: {0}")]
    Lz4Decompress(String),
    /// lz4 压缩失败
    #[error("lz4 compress failed: {0}")]
    Lz4Compress(String),
    /// JPEG 解码失败
    #[error("jpeg decode failed: {0}")]
    JpegDecode(String),
    /// 输入数据不完整
    #[error("incomplete input: need {need} bytes, got {got}")]
    Incomplete {
        /// 需要字节
        need: usize,
        /// 实际字节
        got: usize,
    },
}

/// Codec 结果类型别名
pub type CodecResult<T> = Result<T, CodecError>;

// ============================================================================
// 通用 helper（LCM 与 protobuf 共享）
// ============================================================================

/// lz4 压缩
pub fn lz4_compress(input: &[u8]) -> CodecResult<Bytes> {
    let compressed = lz4_flex::block::compress_prepend_size(input);
    Ok(Bytes::from(compressed))
}

/// lz4 解压
pub fn lz4_decompress(input: &[u8]) -> CodecResult<Bytes> {
    let decompressed = lz4_flex::block::decompress_size_prepended(input)
        .map_err(|e| CodecError::Lz4Decompress(e.to_string()))?;
    Ok(Bytes::from(decompressed))
}

// ============================================================================
// 兼容层：protobuf frame（compat-only,不再作为默认协议层）
// ============================================================================
//
// 总纲修订:protobuf 不再是默认协议层；本节仅保留"确有上游 protobuf 兼容
// 需求"时的 frame 长度前缀工具。请优先使用 `nuwax-protocol::RawLcmFrame`。

/// protobuf 帧头（4 字节大端长度）。**compat-only**,默认协议层已迁移到 LCM。
pub fn write_protobuf_frame(msg: &[u8]) -> BytesMut {
    let len = u32::try_from(msg.len()).expect("protobuf message too large");
    let mut buf = BytesMut::with_capacity(4 + msg.len());
    buf.extend_from_slice(&len.to_be_bytes());
    buf.extend_from_slice(msg);
    buf
}

/// 读取 protobuf 帧头（4 字节大端长度），返回长度与剩余字节。**compat-only**。
pub fn read_protobuf_frame(input: &[u8]) -> CodecResult<(usize, &[u8])> {
    if input.len() < 4 {
        return Err(CodecError::Incomplete {
            need: 4,
            got: input.len(),
        });
    }
    let len_bytes: [u8; 4] = input[..4].try_into().expect("4 bytes");
    let len = u32::from_be_bytes(len_bytes) as usize;
    if input.len() < 4 + len {
        return Err(CodecError::Incomplete {
            need: 4 + len,
            got: input.len(),
        });
    }
    Ok((len, &input[4..4 + len]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lz4_roundtrip() {
        let data = b"hello nuwax, hello world, this is a compression test!";
        let compressed = lz4_compress(data).unwrap();
        // 小数据可能略大于原大小（lz4 frame overhead），但应仍能解压
        let decompressed = lz4_decompress(&compressed).unwrap();
        assert_eq!(decompressed.as_ref(), data);
    }

    #[test]
    fn protobuf_frame_roundtrip_compat() {
        let msg = b"point cloud bytes";
        let frame = write_protobuf_frame(msg);
        let (len, payload) = read_protobuf_frame(&frame).unwrap();
        assert_eq!(len, msg.len());
        assert_eq!(payload, msg);
    }

    #[test]
    fn protobuf_frame_incomplete_compat() {
        let err = read_protobuf_frame(&[1, 2]).unwrap_err();
        assert!(matches!(err, CodecError::Incomplete { need: 4, got: 2 }));
    }
}
