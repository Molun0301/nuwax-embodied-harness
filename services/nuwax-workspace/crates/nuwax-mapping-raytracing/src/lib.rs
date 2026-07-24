// Copyright 2026 Nuwax Team
// SPDX-License-Identifier: Apache-2.0

//! nuwax Mapping Ray Tracing crate.
//!
//! 历史：本 crate 由原 `dimos/mapping/ray_tracing/rust/src/`（lib.rs 6 行 /
//! main.rs 511 行 / voxel_ray_tracer.rs 1318 行 / python.rs 231 行，共 2066 行）
//! 1:1 迁移而来。2026-07-02 评审后拆分：
//!   - `src/voxel_ray_tracer.rs`：1318 行纯 Rust RayCasting/VoxelHashMap 算法，default 暴露。
//!     原 `dimos_module::native_config` 过程宏属性被显式移除（功能等价于"无反射元数据"，
//!     阶段 0 shim 完成后可重新引入）。
//!   - `migration/main.rs`：511 行原始 `dimos-module` + `lcm-msgs` 适配入口，保留为阶段 0 shim 完成后恢复。
//!   - `migration/python.rs`：231 行原始 pyo3 / numpy 绑定，保留为后续启用。
//!
//! 后续启用方式见 `Cargo.toml` features 段注释。

pub mod binding;
pub mod debug;
pub mod transformer;
pub mod voxel_ray_tracer;
