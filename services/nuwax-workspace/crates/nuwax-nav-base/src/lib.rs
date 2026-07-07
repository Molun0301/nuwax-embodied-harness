//! 导航板块"协议面"
//!
//! 定义 trait、错误、生命周期与诊断;**不包含具体 A* / native bridge 算法实现**。
//!
//! ## 模块
//! - [`error`] `NavError` 分层 + `NavResult`
//! - [`traits`] 核心 trait(`GlobalPlanner` / `AsyncGlobalPlanner` / `GoalValidator` /
//!   `PathController` / `GoalManager` / `VisualQueryEngine`)
//! - [`types`] 公共支撑类型(`GoalId` / `GoalTicket` / `GoalValidation` /
//!   `ControllerInput` / `NavStateSnapshot` / `GoalResult` / `GoalManagerOutput`)
//! - [`native`] native runner 相关(`NativeRunnerState` / `NativeProcessError` /
//!   `RestartPolicy` / `ImageView` / `NativeProcessDiagnostics`)
//! - [`diagnostics`] 诊断结构(`NavDiagnostics` / `PlannerDiagnostics` / `ControllerDiagnostics`)
//! - [`mocks`] 各 trait 的 mock 实现(供后续 crate 测试使用)
//!
//! 设计原则:所有公开 trait 的输入/输出只使用 [`nuwax_nav_types_messages`]
//! 或 workspace shared crate 类型,避免依赖具体算法 crate。

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod diagnostics;
pub mod error;
pub mod mocks;
pub mod native;
pub mod traits;
pub mod types;

pub use diagnostics::*;
pub use error::*;
pub use mocks::*;
pub use native::*;
pub use traits::*;
pub use types::*;

/// Prelude:常用导出
pub mod prelude {
    pub use crate::diagnostics::*;
    pub use crate::error::*;
    pub use crate::native::*;
    pub use crate::traits::*;
    pub use crate::types::*;
    pub use nuwax_nav_types_messages::{geometry::*, grid::*, path::*, visual::*, *};
}
