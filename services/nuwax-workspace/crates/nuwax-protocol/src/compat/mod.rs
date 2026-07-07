//! 兼容层（protobuf 边界）
//!
//! 总纲要求 protobuf **不**作为默认协议层；本模块只承载"确有上游 protobuf
//! 兼容需求"的边界工具,例如与 ROS1 bridge 互通、或与已有 `.proto` schema
//! 描述的 C++ 库对接。
//!
//! 当前 P0 foundation milestone:**仅占位**。后续若启用 protobuf 兼容路径,
//! 应在此模块内显式标注,并确保不进入 [`crate::prelude`]。

// 兼容层占位:目前为空模块,避免空集合 re-export 警告下游。
//
// 后续若启用 protobuf,在此处提供 `protobuf_frame::*` 子模块,
//
// 并使用 `#[cfg(feature = "protobuf-compat")]` gate。

#[cfg(test)]
mod tests {
    #[test]
    fn compat_module_is_a_placeholder() {
        // 该模块故意仅占位,验证它能被成功引用即可
        // 用 `1 + 1` 这种必然结果来避免 path_statements / let_unit_value
        // 之类的 clippy 警告,又能让 `cargo test` 跑一个真实 assertion。
        assert_eq!(1 + 1, 2, "占位模块 sanity check");
    }
}
