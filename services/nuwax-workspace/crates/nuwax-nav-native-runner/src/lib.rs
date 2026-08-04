//! native runner 模块
//!
//! 复刻当前 `NativeModule` 的进程生命周期(start / stop / poll / build /
//! rendered_args),为 FAR / Local / PathFollower / PGO / TARE / TerrainAnalysis
//! 等 bridge crate 提供统一启动器。
//!
//! 设计原则:
//! - **不假设 socket IPC,不得硬编码 C++ 路径**(路径由 `NativeProgramSpec::executable` 提供)
//! - CLI 渲染顺序稳定:`BTreeMap` topic 按 port 字典序,然后追加 `args`
//! - 不在 build.rs / 测试中访问网络;构建策略支持 `Never` / `IfMissing` / `Always`
//! - 重启策略支持 `Never` / `OnFailure { max_restarts, backoff_millis }`

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod build;
pub mod rendered_args;
pub mod runner;
pub mod spec;
pub mod supervised;

pub use build::*;
pub use rendered_args::*;
pub use runner::*;
pub use spec::*;
pub use supervised::*;

/// 便捷 re-export 入口,供 `nuwax-nav-stack` 编排 crate 使用。
pub mod prelude {
    pub use crate::build::*;
    pub use crate::rendered_args::*;
    pub use crate::runner::*;
    pub use crate::spec::*;
    pub use nuwax_nav_base::native::*;
}
