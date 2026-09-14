//! `nuwax-explore-wavefront` — 纯 Rust Wavefront Frontier 探索算法
//!
//! ## 公共 API 边界
//!
//! - 公共输出仅为 [`FrontierGoal`](nuwax_nav_types_messages::visual::FrontierGoal)
//! - `FrontierCluster` / `FrontierCandidate` 为 `pub(crate)`(不导出)
//! - `DetectionOutcome` 为 `pub(crate)`(不导出)
//! - `WavefrontError` 公开(返回类型必须 typed)
//! - `WavefrontDetector` **不**独立 re-export;`WavefrontGoalSelector` 是唯一 select owner
//!
//! ## test-internals feature
//!
//! 集成测试访问 crate-private 类型需要 `test-internals` feature。
//! **`default = []`**(R5-1 修复 AC-2 default leak):生产 build 与跨 crate 依赖
//! (`nuwax-exploration --features wavefront`)完全不拉 `test-internals`;
//! `cargo test` 通过 `[dev-dependencies]` 自引用 + `features = ["test-internals"]`
//! 隐式启用,确保测试能访问 `test_helpers`,而生产公共 API 永不暴露。

#![deny(unsafe_code)]
#![warn(missing_docs)]

// 内部模块(全部 crate-private,只在 test-internals feature 下允许 re-export)
mod detector;
mod grid;
mod scoring;
// `selector` 模块对外可见(允许 `nuwax_explore_wavefront::selector::WavefrontGoalSelector`),
// 但模块内 `pub(crate)` 类型仍不进入跨 crate 合同。
pub mod selector;

pub mod classification;
pub mod config;
pub mod diagnostics;
pub mod incremental;
pub mod frontier_goal;
pub mod session;

#[cfg(feature = "legacy-parity")]
pub mod legacy;

#[cfg(feature = "runtime")]
pub mod runtime;

pub use classification::PointClassification;
pub use config::WavefrontConfig;
pub use detector::WavefrontError;
pub use diagnostics::{FrontierDebugSummary, FrontierScoreBreakdown, WavefrontDiagnostics};
pub use frontier_goal::{FrontierGoalIdSeed, FrontierGoalIdSource, GoalSelection, StopReason};
pub use grid::GridPoint;
pub use selector::WavefrontGoalSelector;
pub use session::{Direction2, ExplorationSession};

#[doc(hidden)]
#[cfg(any(test, feature = "test-internals"))]
pub mod test_helpers;
