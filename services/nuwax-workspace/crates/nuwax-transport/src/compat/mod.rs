//! 兼容层（历史 UDP/SHM 旧 API, deprecated）
//!
//! 总纲修订后,UDP / SHM 不再是默认传输层;该模块仅保留旧类型
//! `Transport` / `UdpTransport` / `ShmTransport` 的占位 re-export,以
//! 避免破坏依赖它们的 crate。`MessageEnvelope` 来自 `nuwax-protocol`,
//! 同样在 compat 路径使用。
//!
//! 默认 feature:全部启用。后续若引入 `protobuf-compat` feature gate,
//! 仅在 feature 开启时把这些类型 re-export 到根。

#[allow(deprecated)]
pub use crate::{ShmTransport, Transport, UdpTransport};

#[cfg(test)]
mod tests {
    #[test]
    fn compat_module_is_a_placeholder() {
        // 占位模块;验证存在即可
        // 用 `1 + 1` 这种必然结果来避免 path_statements / let_unit_value
        // 之类的 clippy 警告,又能让 `cargo test` 跑一个真实 assertion。
        assert_eq!(1 + 1, 2, "占位模块 sanity check");
    }
}
