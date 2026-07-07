//! protobuf 兼容层
//!
//! **compat-only** —— 总纲修订后,protobuf 不再作为默认协议层。
//! 本模块仅承载 frame 长度前缀工具,集中 `nuwax-codec` 旧 API;
//! 业务层应使用 [`crate::lcm`] 与 [`nuwax_protocol::RawLcmFrame`] 替代。
//!
//! `nuwax-codec::write_protobuf_frame` / `nuwax-codec::read_protobuf_frame`
//! 重新从这里 re-export,作为 compat path 入口。
//!
//! 默认 feature:全部启用。后续若引入 `protobuf-compat` feature gate,
//! 仅在 feature 开启时把本模块暴露在 `prelude` 中。

pub use crate::{read_protobuf_frame, write_protobuf_frame};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn protobuf_frame_roundtrip_compat() {
        let payload = b"some-bytes";
        let frame = write_protobuf_frame(payload);
        let (len, body) = read_protobuf_frame(&frame).unwrap();
        assert_eq!(len, payload.len());
        assert_eq!(body, payload);
    }
}
